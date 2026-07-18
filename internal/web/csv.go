package web

import (
	"bufio"
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/xet-labs/roboarm-commander/internal/arm"
)

// exportCSV / importCSV convert at the file boundary: internal Step
// storage is deg10 (matches wire protocol), but the CSV interchange
// format is plain degrees — matching the original project dataset
// shape from the brief (`time,s0,s1,s2,s3` / `0012,68,70,50,39`),
// which is also just friendlier to read/edit by hand.
//
//	time,s0,s1,s2,s3
//	0,90.0,90.0,90.0,90.0
//	120,91.2,90.0,88.5,90.0
//	...
func exportCSV(w io.Writer, steps []arm.Step) error {
	bw := bufio.NewWriter(w)
	defer bw.Flush()

	if _, err := fmt.Fprintln(bw, "time,s0,s1,s2,s3"); err != nil {
		return err
	}
	for _, s := range steps {
		if _, err := fmt.Fprintf(bw, "%d,%.1f,%.1f,%.1f,%.1f\n",
			s.TMs,
			float64(s.Angles[0])/10.0,
			float64(s.Angles[1])/10.0,
			float64(s.Angles[2])/10.0,
			float64(s.Angles[3])/10.0); err != nil {
			return err
		}
	}
	return nil
}

func importCSV(r io.Reader) ([]arm.Step, error) {
	sc := bufio.NewScanner(r)
	var steps []arm.Step
	lineNo := 0

	const degMin, degMax = 0.0, 180.0 // human-readable bound at the CSV boundary

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
			v, err := strconv.ParseFloat(strings.TrimSpace(fields[i+1]), 64)
			if err != nil {
				return nil, fmt.Errorf("csv line %d: bad angle value: %w", lineNo, err)
			}
			if v < degMin || v > degMax {
				return nil, fmt.Errorf("csv line %d: angle %.1f out of range [%.0f,%.0f]", lineNo, v, degMin, degMax)
			}
			step.Angles[i] = int16(v * 10.0)
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
