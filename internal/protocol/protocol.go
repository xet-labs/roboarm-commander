// Package protocol defines the RPi<->ESP32 UART frame format.
//
// This is the single source of truth for the wire format — the ESP32
// firmware's frame parser (not yet written) MUST match this exactly.
// Keep both sides pointed at this file's doc comment when implementing
// the C++ side.
//
// Frame layout:
//
//	byte 0    : 0xAA           magic 1
//	byte 1    : 0x55           magic 2
//	byte 2    : LEN            = 2 + len(payload)   (covers CMD + JOINT_ID + payload)
//	byte 3    : CMD
//	byte 4    : JOINT_ID       (0xFF = "not applicable / broadcast")
//	byte 5..  : PAYLOAD        (LEN-2 bytes)
//	last byte : CHECKSUM       = XOR of every byte from LEN through the last payload byte
//
// Commands (request, host -> ESP32):
//
//	CmdMove     (0x01)  JOINT_ID=0..3, payload=[angleDeg uint8]
//	CmdClaw     (0x02)  JOINT_ID=0xFF, payload=[direction int8 as uint8, dutyPWM uint8]
//	CmdStop     (0x03)  JOINT_ID=0xFF, payload=[]
//	CmdGetState (0x04)  JOINT_ID=0xFF, payload=[]
//
// Response (ESP32 -> host), only ever sent after CmdGetState:
//
//	CmdState    (0x84)  JOINT_ID=0xFF,
//	                     payload=[angleBase, angleShoulder, angleElbow, angleWrist (uint8 each),
//	                              clawCurrentMa (uint16 LE)]
package protocol

import (
	"bufio"
	"fmt"
)

const (
	Magic1 = 0xAA
	Magic2 = 0x55

	JointAll = 0xFF // JOINT_ID value meaning "not a per-joint command"
)

type Cmd byte

const (
	CmdMove     Cmd = 0x01
	CmdClaw     Cmd = 0x02
	CmdStop     Cmd = 0x03
	CmdGetState Cmd = 0x04
	CmdState    Cmd = 0x84 // response bit (0x80) set over CmdGetState
)

// Joint indices — MUST match JointId enum order in firmware joints.h.
const (
	JointBase     byte = 0
	JointShoulder byte = 1
	JointElbow    byte = 2
	JointWrist    byte = 3
)

type Frame struct {
	Cmd     Cmd
	JointID byte
	Payload []byte
}

// Encode serializes a Frame to wire bytes, including sync, length and checksum.
func Encode(f Frame) []byte {
	length := byte(2 + len(f.Payload))
	buf := make([]byte, 0, 3+int(length)+1)
	buf = append(buf, Magic1, Magic2, length, byte(f.Cmd), f.JointID)
	buf = append(buf, f.Payload...)

	var cksum byte
	for _, b := range buf[2:] { // XOR from LEN through last payload byte
		cksum ^= b
	}
	buf = append(buf, cksum)
	return buf
}

// MoveFrame is a convenience constructor for the common case.
func MoveFrame(jointID byte, angleDeg int) Frame {
	if angleDeg < 0 {
		angleDeg = 0
	}
	if angleDeg > 255 {
		angleDeg = 255
	}
	return Frame{Cmd: CmdMove, JointID: jointID, Payload: []byte{byte(angleDeg)}}
}

func ClawFrame(direction int8, duty byte) Frame {
	return Frame{Cmd: CmdClaw, JointID: JointAll, Payload: []byte{byte(direction), duty}}
}

func StopFrame() Frame {
	return Frame{Cmd: CmdStop, JointID: JointAll}
}

func GetStateFrame() Frame {
	return Frame{Cmd: CmdGetState, JointID: JointAll}
}

// State is the decoded payload of a CmdState response.
type State struct {
	Angles        [4]int // Base, Shoulder, Elbow, Wrist
	ClawCurrentMa int
}

func DecodeState(f Frame) (State, error) {
	if f.Cmd != CmdState {
		return State{}, fmt.Errorf("protocol: not a state frame (cmd=0x%02x)", byte(f.Cmd))
	}
	if len(f.Payload) < 6 {
		return State{}, fmt.Errorf("protocol: state payload too short (%d bytes)", len(f.Payload))
	}
	s := State{}
	for i := 0; i < 4; i++ {
		s.Angles[i] = int(f.Payload[i])
	}
	s.ClawCurrentMa = int(f.Payload[4]) | int(f.Payload[5])<<8
	return s, nil
}

// ReadFrame blocks until a complete, checksum-valid frame is read from r,
// or an error occurs. Bytes that don't start a valid frame are discarded
// one at a time (resync-on-garbage), which matters on a shared UART where
// the ESP32 may also emit plain-text debug logging — this reader silently
// skips anything that isn't a well-formed frame rather than erroring out.
func ReadFrame(r *bufio.Reader) (Frame, error) {
	for {
		b1, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		if b1 != Magic1 {
			continue
		}
		b2, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		if b2 != Magic2 {
			continue // false positive on 0xAA, resync
		}

		length, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}
		if length < 2 {
			continue // malformed, resync
		}

		body := make([]byte, length) // CMD + JOINT_ID + payload
		if _, err := ioReadFull(r, body); err != nil {
			return Frame{}, err
		}

		cksumByte, err := r.ReadByte()
		if err != nil {
			return Frame{}, err
		}

		var want byte = length
		for _, b := range body {
			want ^= b
		}
		if want != cksumByte {
			// checksum mismatch — drop this frame and keep scanning,
			// don't propagate as a fatal error (one corrupted frame
			// shouldn't kill the read loop)
			continue
		}

		return Frame{
			Cmd:     Cmd(body[0]),
			JointID: body[1],
			Payload: body[2:],
		}, nil
	}
}

func ioReadFull(r *bufio.Reader, buf []byte) (int, error) {
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
