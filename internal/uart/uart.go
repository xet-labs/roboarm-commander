// Package uart wraps the serial link to the ESP32 controller.
package uart

import (
	"bufio"
	"fmt"
	"log"
	"sync"

	"go.bug.st/serial"

	"github.com/xet-labs/roboarm-commander/internal/protocol"
)

type Link struct {
	port   serial.Port
	reader *bufio.Reader

	writeMu sync.Mutex

	frames chan protocol.Frame
	closed chan struct{}
}

// Open connects to the ESP32 over UART. portName is e.g. "/dev/ttyUSB0" or
// "/dev/ttyACM0" — auto-detect isn't worth the complexity for a bench setup
// where the port is stable; pass it explicitly via config/flag.
func Open(portName string, baud int) (*Link, error) {
	mode := &serial.Mode{BaudRate: baud}
	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("uart: open %s: %w", portName, err)
	}

	l := &Link{
		port:   port,
		reader: bufio.NewReader(port),
		frames: make(chan protocol.Frame, 16),
		closed: make(chan struct{}),
	}

	go l.readLoop()
	return l, nil
}

func (l *Link) readLoop() {
	for {
		f, err := protocol.ReadFrame(l.reader)
		if err != nil {
			select {
			case <-l.closed:
				return
			default:
			}
			log.Printf("[uart] read error (will keep retrying): %v", err)
			continue
		}
		select {
		case l.frames <- f:
		default:
			log.Printf("[uart] frame channel full, dropping oldest")
			<-l.frames
			l.frames <- f
		}
	}
}

// Frames delivers every successfully decoded frame from the ESP32,
// including CmdState responses. Callers needing a specific response
// (e.g. GetState round-trip) should drain this channel with a select
// against a timeout — see arm.Arm.pollState for the pattern.
func (l *Link) Frames() <-chan protocol.Frame {
	return l.frames
}

func (l *Link) Send(f protocol.Frame) error {
	l.writeMu.Lock()
	defer l.writeMu.Unlock()
	_, err := l.port.Write(protocol.Encode(f))
	if err != nil {
		return fmt.Errorf("uart: write: %w", err)
	}
	return nil
}

func (l *Link) Close() error {
	close(l.closed)
	return l.port.Close()
}
