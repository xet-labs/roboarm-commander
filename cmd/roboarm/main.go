package main

import (
	"flag"
	"log"

	"github.com/xet-labs/roboarm-commander/internal/arm"
	"github.com/xet-labs/roboarm-commander/internal/store"
	"github.com/xet-labs/roboarm-commander/internal/uart"
	"github.com/xet-labs/roboarm-commander/internal/web"
	"github.com/xet-labs/roboarm-commander/internal/xbox"
)

func main() {
	uartPort := flag.String("uart", "/dev/ttyUSB0", "serial port to the ESP32 controller")
	uartBaud := flag.Int("baud", 115200, "UART baud rate — must match roboArm.ino Serial.begin()")
	dbPath := flag.String("db", "roboarm.db", "SQLite database path")
	webAddr := flag.String("web", ":8080", "web UI/API listen address")
	xboxAddr := flag.String("xbox", "127.0.0.1:9999", "UDP address the xbox_bridge.py forwards to")
	flag.Parse()

	link, err := uart.Open(*uartPort, *uartBaud)
	if err != nil {
		log.Fatalf("uart: %v", err)
	}
	defer link.Close()

	st, err := store.Open(*dbPath)
	if err != nil {
		log.Fatalf("store: %v", err)
	}
	defer st.Close()

	a := arm.New(link)

	if err := xbox.Listen(*xboxAddr, a); err != nil {
		log.Fatalf("xbox: %v", err)
	}

	srv := web.New(a, st)
	log.Fatal(srv.ListenAndServe(*webAddr))
}
