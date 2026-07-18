package protocol

import (
	"bufio"
	"bytes"
	"testing"
)

// TestMoveJointExactBytes hand-verifies the wire encoding against the
// firmware's documented algorithm (protocol.h / protocol.cpp), byte for
// byte, since there's no C++ toolchain in this build environment to
// cross-compile and test directly against. jointId=1 (Shoulder),
// posDeg10=900 (90.0deg), timeMs=1000.
//
// payload = [0x01, 0x84,0x03 (900 LE), 0xE8,0x03 (1000 LE)]
// LEN = 1(cmd) + 5(payload) = 6
// checksum = LEN ^ CMD ^ payload... = 6^1^1^0x84^0x03^0xE8^0x03 = 0x6A
func TestMoveJointExactBytes(t *testing.T) {
	f := MoveJointFrame(JointShoulder, 900, 1000)
	got, err := Encode(f)
	if err != nil {
		t.Fatalf("encode: %v", err)
	}
	want := []byte{0xAA, 0x55, 0x06, 0x01, 0x01, 0x84, 0x03, 0xE8, 0x03, 0x6A}
	if !bytes.Equal(got, want) {
		t.Fatalf("byte mismatch:\n got  % X\n want % X", got, want)
	}
}

func TestEncodeDecodeRoundTrip(t *testing.T) {
	cases := []Frame{
		MoveJointFrame(JointBase, -300, 250),
		MoveAllFrame([JointCount]int16{0, 1800, 900, 450}, 2000),
		ClawSetFrame(1, 200),
		StopFrame(),
		GetStateFrame(),
		SetTorqueFrame(JointElbow, true),
		PingFrame(0xDEADBEEF),
	}

	for i, want := range cases {
		wire, err := Encode(want)
		if err != nil {
			t.Fatalf("case %d: encode: %v", i, err)
		}
		got, err := ReadFrame(bufio.NewReader(bytes.NewReader(wire)))
		if err != nil {
			t.Fatalf("case %d: readframe: %v", i, err)
		}
		if got.Cmd != want.Cmd {
			t.Fatalf("case %d: cmd mismatch: got 0x%02x want 0x%02x", i, got.Cmd, want.Cmd)
		}
		if !bytes.Equal(got.Payload, want.Payload) {
			t.Fatalf("case %d: payload mismatch: got % X want % X", i, got.Payload, want.Payload)
		}
	}
}

func TestReadFrameResyncsPastGarbageAndBadChecksum(t *testing.T) {
	good, _ := Encode(StopFrame())

	corrupt, _ := Encode(GetStateFrame())
	corrupt[len(corrupt)-1] ^= 0xFF // wreck the checksum

	// noise, then a repeated-0xAA run (must not desync the parser),
	// then a corrupted frame (must be skipped), then a valid frame.
	stream := append([]byte{0x00, 0xFF, 0xAA, 0xAA, 0xAA}, corrupt...)
	stream = append(stream, good...)

	r := bufio.NewReader(bytes.NewReader(stream))
	got, err := ReadFrame(r)
	if err != nil {
		t.Fatalf("readframe: %v", err)
	}
	if got.Cmd != CmdStop {
		t.Fatalf("expected to skip corrupt frame and land on CmdStop, got 0x%02x", got.Cmd)
	}
}

func TestDecodeState(t *testing.T) {
	payload := make([]byte, 15)
	// angles: 0, 900, 1800, -50 (deg10)
	angles := [4]int16{0, 900, 1800, -50}
	for i, a := range angles {
		payload[i*2] = byte(uint16(a))
		payload[i*2+1] = byte(uint16(a) >> 8)
	}
	payload[8] = 0xE8 // clawCurrentMa LE = 1000
	payload[9] = 0x03
	payload[10] = 1 // clawMode = close
	payload[11] = 0x40
	payload[12] = 0x0D
	payload[13] = 0x03
	payload[14] = 0x00 // uptimeMs LE = 200000

	s, err := DecodeState(Frame{Cmd: RspState, Payload: payload})
	if err != nil {
		t.Fatalf("decode: %v", err)
	}
	if s.AnglesDeg10 != angles {
		t.Fatalf("angles mismatch: got %v want %v", s.AnglesDeg10, angles)
	}
	if s.ClawCurrentMa != 1000 {
		t.Fatalf("clawCurrentMa mismatch: got %d want 1000", s.ClawCurrentMa)
	}
	if s.ClawMode != 1 {
		t.Fatalf("clawMode mismatch: got %d want 1", s.ClawMode)
	}
	if s.UptimeMs != 200000 {
		t.Fatalf("uptimeMs mismatch: got %d want 200000", s.UptimeMs)
	}
}
