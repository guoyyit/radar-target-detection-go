package radar

import (
	"context"
	"path/filepath"
	"testing"
)

func simFrame(id int64, seed int64) RawFrame {
	raw := SimulateFrame(SimConfig{
		Header: testHeader(),
		Targets: []SimTarget{
			{Range: 12, Velocity: 3.5, Amplitude: 10},
			{Range: 30, Velocity: -6, Amplitude: 8},
		},
		NoiseSigma: 0.4, Seed: seed,
	})
	raw.ID = id
	return raw
}

func TestRestartNoDuplicateOrSkippedFrame(t *testing.T) {
	path := filepath.Join(t.TempDir(), "ckpt.json")
	store, err := NewFileCheckpointStore(path)
	if err != nil {
		t.Fatal(err)
	}
	cfg := testConfig()

	// Session 1: process frames 1..3.
	p1, err := NewProcessor(cfg, store)
	if err != nil {
		t.Fatal(err)
	}
	for id := int64(1); id <= 3; id++ {
		if _, err := p1.ProcessFrame(simFrame(id, 100+id)); err != nil {
			t.Fatal(err)
		}
	}

	// Restart: replay 1..5. Frames 1..3 must be skipped as stale, 4..5 processed.
	p2, err := NewProcessor(cfg, store)
	if err != nil {
		t.Fatal(err)
	}
	if p2.LastCompletedFrame() != 3 {
		t.Fatalf("restart position = %d, want 3", p2.LastCompletedFrame())
	}
	for id := int64(1); id <= 3; id++ {
		if _, err := p2.ProcessFrame(simFrame(id, 100+id)); err != ErrFrameStale {
			t.Fatalf("frame %d replay: got %v want ErrFrameStale", id, err)
		}
	}
	for id := int64(4); id <= 5; id++ {
		if _, err := p2.ProcessFrame(simFrame(id, 100+id)); err != nil {
			t.Fatalf("frame %d after restart: %v", id, err)
		}
	}
	if p2.LastCompletedFrame() != 5 {
		t.Fatalf("final position = %d, want 5", p2.LastCompletedFrame())
	}

	// Gap must be rejected rather than silently skipping.
	if _, err := p2.ProcessFrame(simFrame(7, 107)); err != ErrFrameOutOfOrder {
		t.Fatalf("gap frame: got %v want ErrFrameOutOfOrder", err)
	}
}

func TestRejectedFrameAdvancesCheckpoint(t *testing.T) {
	store := NewMemoryCheckpointStore()
	cfg := testConfig()
	p, err := NewProcessor(cfg, store)
	if err != nil {
		t.Fatal(err)
	}
	bad := simFrame(1, 1)
	bad.Header.PulseCount = 1
	res, err := p.ProcessFrame(bad)
	if err != nil {
		t.Fatal(err)
	}
	if !res.Rejected || res.Reason != ParseBadPulseCount {
		t.Fatalf("want rejected BadPulseCount, got %+v", res)
	}
	if p.LastCompletedFrame() != 1 {
		t.Fatalf("checkpoint should advance for rejected frames, got %d", p.LastCompletedFrame())
	}
}

func TestDeterministicOutputAcrossRuns(t *testing.T) {
	cfg := testConfig()
	run := func(n int64) []FrameResult {
		p, err := NewProcessor(cfg, NewMemoryCheckpointStore())
		if err != nil {
			t.Fatal(err)
		}
		var out []FrameResult
		for id := int64(1); id <= n; id++ {
			res, err := p.ProcessFrame(simFrame(id, 100+id))
			if err != nil {
				t.Fatal(err)
			}
			out = append(out, *res)
		}
		return out
	}
	a, b := run(5), run(5)
	if len(a) != len(b) {
		t.Fatal("run length differs")
	}
	for i := range a {
		if !resultsEqual(a[i], b[i]) {
			t.Fatalf("frame %d output differs between runs", a[i].FrameID)
		}
	}
}

func resultsEqual(x, y FrameResult) bool {
	if x.FrameID != y.FrameID || x.Rejected != y.Rejected {
		return false
	}
	if len(x.Detections) != len(y.Detections) || len(x.Targets) != len(y.Targets) {
		return false
	}
	for i := range x.Detections {
		if x.Detections[i] != y.Detections[i] {
			return false
		}
	}
	for i := range x.Targets {
		if x.Targets[i] != y.Targets[i] {
			return false
		}
	}
	return true
}

func TestRunSkipsStaleFramesFromPipe(t *testing.T) {
	path := filepath.Join(t.TempDir(), "ckpt.json")
	store, err := NewFileCheckpointStore(path)
	if err != nil {
		t.Fatal(err)
	}
	cfg := testConfig()
	p1, err := NewProcessor(cfg, store)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := p1.ProcessFrame(simFrame(1, 101)); err != nil {
		t.Fatal(err)
	}

	p2, err := NewProcessor(cfg, store)
	if err != nil {
		t.Fatal(err)
	}
	pipe := NewPipe(4)
	pipe.Push(simFrame(1, 101)) // stale after restart
	pipe.Push(simFrame(2, 102))
	pipe.Close()
	var seen []int64
	err = p2.Run(context.Background(), pipe, func(r FrameResult) {
		seen = append(seen, r.FrameID)
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(seen) != 1 || seen[0] != 2 {
		t.Fatalf("emitted frames = %v, want only [2]", seen)
	}
}
