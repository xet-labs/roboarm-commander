// Package protocol defines the RPi<->ESP32 UART frame format.
//
// This MUST match roboArm_controller/protocol.h on the firmware side
// byte-for-byte. It does, as of this file — if you change one side,
// change the other and re-check both.
//
// Frame layout:
//
//	byte 0    : 0xAA           magic 1
//	byte 1    : 0x55           magic 2
//	byte 2    : LEN            = 1 + len(payload)   (covers CMD + payload)
//	byte 3    : CMD
//	byte 4..  : PAYLOAD        (LEN-1 bytes)
//	last byte : CHECKSUM       = XOR of LEN, CMD, and every payload byte
//
// There is no dedicated JOINT_ID frame field — for CmdMoveJoint the
// joint id is the first payload byte; CmdMoveAll addresses all 4
// joints implicitly via payload position.
package protocol

import (
	"bufio"
	"encoding/binary"
	"fmt"
)

const (
	Magic0 = 0xAA
	Magic1 = 0x55

	MaxPayload = 32
)

type Cmd byte

const (
	// Host -> Controller
	CmdMoveJoint Cmd = 0x01 // u8 jointId, i16 posDeg10, u16 timeMs   (5B)
	CmdMoveAll   Cmd = 0x02 // i16 pos[4]Deg10, u16 timeMs            (10B)
	CmdClawSet   Cmd = 0x03 // u8 mode(0=stop,1=close,2=open), u8 pwm (2B)
	CmdStop      Cmd = 0x04 // (0B)
	CmdGetState  Cmd = 0x05 // (0B) -> triggers RspState
	CmdSetTorque Cmd = 0x06 // u8 jointId, u8 enable                  (2B)
	CmdPing      Cmd = 0x07 // u32 echoToken                          (4B)

	// Controller -> Host
	RspState Cmd = 0x81 // i16 pos[4]Deg10, u16 clawCurrentMa, u8 clawMode, u32 uptimeMs (15B)
	RspPong  Cmd = 0x82 // u32 echoToken
	RspAck   Cmd = 0x83 // u8 cmdEcho
	RspErr   Cmd = 0x84 // u8 cmdEcho, u8 errCode
)

// Joint indices — MUST match JointId enum order in firmware config.h.
const (
	JointBase     byte = 0
	JointShoulder byte = 1
	JointElbow    byte = 2
	JointWrist    byte = 3
	JointCount         = 4
)

type ErrCode byte

const (
	ErrNone        ErrCode = 0
	ErrBadChecksum ErrCode = 1
	ErrBadLength   ErrCode = 2
	ErrUnknownCmd  ErrCode = 3
	ErrBadJoint    ErrCode = 4
	ErrQueueFull   ErrCode = 5
)

type Frame struct {
	Cmd     Cmd
	Payload []byte
}

// checksum matches firmware's proto::checksum(len, cmd, payload, payloadLen).
func checksum(length byte, cmd Cmd, payload []byte) byte {
	c := length ^ byte(cmd)
	for _, b := range payload {
		c ^= b
	}
	return c
}

// Encode serializes a Frame to wire bytes, including sync, length and checksum.
func Encode(f Frame) ([]byte, error) {
	if len(f.Payload) > MaxPayload {
		return nil, fmt.Errorf("protocol: payload too large (%d > %d)", len(f.Payload), MaxPayload)
	}
	length := byte(1 + len(f.Payload))

	buf := make([]byte, 0, 3+int(length)+1)
	buf = append(buf, Magic0, Magic1, length, byte(f.Cmd))
	buf = append(buf, f.Payload...)
	buf = append(buf, checksum(length, f.Cmd, f.Payload))
	return buf, nil
}

// --- convenience constructors, one per command --------------------------

func MoveJointFrame(jointID byte, posDeg10 int16, timeMs uint16) Frame {
	p := make([]byte, 5)
	p[0] = jointID
	binary.LittleEndian.PutUint16(p[1:3], uint16(posDeg10))
	binary.LittleEndian.PutUint16(p[3:5], timeMs)
	return Frame{Cmd: CmdMoveJoint, Payload: p}
}

// MoveAllFrame — pos must be [JointBase, JointShoulder, JointElbow, JointWrist] order.
func MoveAllFrame(pos [JointCount]int16, timeMs uint16) Frame {
	p := make([]byte, 10)
	for i, v := range pos {
		binary.LittleEndian.PutUint16(p[i*2:i*2+2], uint16(v))
	}
	binary.LittleEndian.PutUint16(p[8:10], timeMs)
	return Frame{Cmd: CmdMoveAll, Payload: p}
}

// ClawMode: 0=stop 1=close 2=open — matches firmware's Claw::setMode().
func ClawSetFrame(mode byte, pwm byte) Frame {
	return Frame{Cmd: CmdClawSet, Payload: []byte{mode, pwm}}
}

func StopFrame() Frame {
	return Frame{Cmd: CmdStop}
}

func GetStateFrame() Frame {
	return Frame{Cmd: CmdGetState}
}

func SetTorqueFrame(jointID byte, enable bool) Frame {
	e := byte(0)
	if enable {
		e = 1
	}
	return Frame{Cmd: CmdSetTorque, Payload: []byte{jointID, e}}
}

func PingFrame(token uint32) Frame {
	p := make([]byte, 4)
	binary.LittleEndian.PutUint32(p, token)
	return Frame{Cmd: CmdPing, Payload: p}
}

// --- response decoding ----------------------------------------------------

// State is the decoded payload of an RspState frame. Angles are in
// tenths of a degree (deg10), matching the wire format — divide by 10
// for a human-readable value.
type State struct {
	AnglesDeg10   [JointCount]int16
	ClawCurrentMa uint16
	ClawMode      byte
	UptimeMs      uint32
}

func DecodeState(f Frame) (State, error) {
	if f.Cmd != RspState {
		return State{}, fmt.Errorf("protocol: not a state frame (cmd=0x%02x)", byte(f.Cmd))
	}
	if len(f.Payload) < 15 {
		return State{}, fmt.Errorf("protocol: state payload too short (%d bytes)", len(f.Payload))
	}
	var s State
	for i := 0; i < JointCount; i++ {
		s.AnglesDeg10[i] = int16(binary.LittleEndian.Uint16(f.Payload[i*2 : i*2+2]))
	}
	s.ClawCurrentMa = binary.LittleEndian.Uint16(f.Payload[8:10])
	s.ClawMode = f.Payload[10]
	s.UptimeMs = binary.LittleEndian.Uint32(f.Payload[11:15])
	return s, nil
}

type AckOrErr struct {
	IsErr   bool
	CmdEcho byte
	ErrCode ErrCode
}

func DecodeAckOrErr(f Frame) (AckOrErr, error) {
	switch f.Cmd {
	case RspAck:
		if len(f.Payload) < 1 {
			return AckOrErr{}, fmt.Errorf("protocol: ack payload too short")
		}
		return AckOrErr{IsErr: false, CmdEcho: f.Payload[0]}, nil
	case RspErr:
		if len(f.Payload) < 2 {
			return AckOrErr{}, fmt.Errorf("protocol: err payload too short")
		}
		return AckOrErr{IsErr: true, CmdEcho: f.Payload[0], ErrCode: ErrCode(f.Payload[1])}, nil
	default:
		return AckOrErr{}, fmt.Errorf("protocol: not an ack/err frame (cmd=0x%02x)", byte(f.Cmd))
	}
}

// ReadFrame blocks until a complete, checksum-valid frame is read from r,
// or an error occurs. Bytes that don't start a valid frame are discarded
// one at a time (resync-on-garbage), which matters on a shared UART where
// the ESP32 may also emit plain-text debug logging on its OWN USB serial
// — not this link, but the same defensive parsing applies if noise ever
// gets on the wire.
func ReadFrame(r *bufio.Reader) (Frame, error) {
	for {
		b0, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		if b0 != Magic0 {
			continue
		}

		b1, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		for b1 == Magic0 {
			// absorb repeated 0xAA — keep waiting for the 0x55 that follows,
			// same effect as firmware's parser staying in WAIT_M1 on repeats
			b1, err = r.ReadByte()
			if err != nil {
				return Frame{}, err
			}
		}
		if b1 != Magic1 {
			continue // not a valid magic pair, resync from scratch
		}

		length, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		if length == 0 || length > (MaxPayload+1) {
			continue // malformed, resync
		}

		body := make([]byte, length) // CMD + payload
		if _, err := readFull(r, body); err != nil {
			return Frame{}, err
		}

		cksumByte, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}

		want := checksum(length, Cmd(body[0]), body[1:])
		if want != cksumByte {
			// checksum mismatch — drop this frame and keep scanning,
			// don't propagate as a fatal error (one corrupted frame
			// shouldn't kill the read loop)
			continue
		}

		return Frame{Cmd: Cmd(body[0]), Payload: body[1:]}, nil
	}
}

func readFull(r *bufio.Reader, buf []byte) (int, error) {
	n := 0
	for n < len(buf) {
		m, err := r.Read(buf[n:])
		n += m
		if err != nil {
			return n, err
		}
	}
	return n, nil
}
