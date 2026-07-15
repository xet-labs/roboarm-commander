package web

import (
	"bufio"
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/xet-labs/roboarm-commander/internal/arm"
)

// exportCSV writes steps in the project's original dataset shape:
//
//	time,s0,s1,s2,s3
//	0,90,90,90,90
//	120,91,90,89,90
//	...
func exportCSV(w io.Writer, steps []arm.Step) error {
	bw := bufio.NewWriter(w)
	defer bw.Flush()

	if _, err := fmt.Fprintln(bw, "time,s0,s1,s2,s3"); err != nil {
		return err
	}
	for _, s := range steps {
		if _, err := fmt.Fprintf(bw, "%d,%d,%d,%d,%d\n",
			s.TMs, s.Angles[0], s.Angles[1], s.Angles[2], s.Angles[3]); err != nil {
			return err
		}
	}
	return nil
}

func importCSV(r io.Reader) ([]arm.Step, error) {
	sc := bufio.NewScanner(r)
	var steps []arm.Step
	lineNo := 0

	for sc.Scan() {
		lineNo++
		line := strings.TrimSpace(sc.Text())
		if line == "" {
			continue
		}
		if lineNo == 1 && strings.HasPrefix(strings.ToLower(line), "time,") {
			continue // header
		}

		fields := strings.Split(line, ",")
		if len(fields) != 5 {
			return nil, fmt.Errorf("csv line %d: expected 5 fields, got %d", lineNo, len(fields))
		}

		t, err := strconv.ParseInt(strings.TrimSpace(fields[0]), 10, 64)
		if err != nil {
			return nil, fmt.Errorf("csv line %d: bad time value: %w", lineNo, err)
		}

		var step arm.Step
		step.TMs = t
		for i := 0; i < 4; i++ {
			v, err := strconv.Atoi(strings.TrimSpace(fields[i+1]))
			if err != nil {
				return nil, fmt.Errorf("csv line %d: bad angle value: %w", lineNo, err)
			}
			if v < arm.AngleMin || v > arm.AngleMax {
				return nil, fmt.Errorf("csv line %d: angle %d out of range [%d,%d]", lineNo, v, arm.AngleMin, arm.AngleMax)
			}
			step.Angles[i] = v
		}
		steps = append(steps, step)
	}

	if err := sc.Err(); err != nil {
		return nil, err
	}
	if len(steps) == 0 {
		return nil, fmt.Errorf("csv: no data rows found")
	}
	return steps, nil
}
