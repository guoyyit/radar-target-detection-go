package pipeline

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"runtime"
	"testing"
	"time"

	"github.com/guoyyit/radar-target-detection-go/pkg/detect"
	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
	"github.com/guoyyit/radar-target-detection-go/pkg/signal"
	"github.com/guoyyit/radar-target-detection-go/pkg/sim"
	"github.com/guoyyit/radar-target-detection-go/pkg/track"
)

func mustRead(t *testing.T, p string) []byte {
	t.Helper()
	b, err := os.ReadFile(p)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func testConfig(path string) Config {
	return Config{
		Parse: frame.ParseConfig{},
		Signal: signal.Config{
			Window:         signal.WindowHann,
			RemoveDC:       true,
			RangeZeroPad:   2,
			DopplerZeroPad: 2,
		},
		CFAR: detect.CFARConfig{
			GuardRange: 2, GuardDoppler: 2,
			TrainRange: 4, TrainDoppler: 4,
			AlphaDB: 12, SkipRangeDC: true, SkipNearBins: 3,
		},
		Cluster: detect.ClusterConfig{
			MaxRangeGap: 1, MaxDopplerGap: 1, MinCells: 2, Interpolate: true,
		},
		Track: track.Config{
			RangeGate: 3, VelocityGate: 1.5,
			ConfirmCount: 2, HoldFrames: 2, WindowLength: 3,
		},
		CheckpointPath: path,
	}
}

func simGen(t *testing.T) *sim.Generator {
	t.Helper()
	h := frame.Header{
		SampleRate: 10e6, ChirpSlope: 12e12, CarrierFreq: 77e9,
		PulseInterval: 40e-6, PulseCount: 32, SamplesPerPulse: 128,
	}
	g, err := sim.New(sim.Config{
		Header: h,
		Targets: []sim.TargetSpec{
			{Range: 20, Velocity: 1, Amplitude: 8},
			{Range: 40, Velocity: -2, Amplitude: 8},
		},
		NoiseSigma: 0.05,
		DCOffsetI:  3,
		Seed:       1234,
	})
	if err != nil {
		t.Fatal(err)
	}
	return g
}

func runFrames(t *testing.T, cfg Config, seqs []int64) []FrameResult {
	t.Helper()
	svc, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	in := make(chan []byte, len(seqs))
	out := make(chan FrameResult, len(seqs))
	g := simGen(t)
	for _, s := range seqs {
		raw, err := g.EncodedFrame(s)
		if err != nil {
			t.Fatal(err)
		}
		in <- raw
	}
	close(in)
	if err := svc.Run(context.Background(), in, out); err != nil {
		t.Fatal(err)
	}
	var res []FrameResult
	for r := range out {
		res = append(res, r)
	}
	return res
}

func TestEndToEndDetectsTargets(t *testing.T) {
	res := runFrames(t, testConfig(filepath.Join(t.TempDir(), "cp")), []int64{0, 1, 2, 3})
	if len(res) != 4 {
		t.Fatalf("want 4 results, got %d", len(res))
	}
	found := false
	for _, r := range res[2:] {
		for _, tg := range r.Targets {
			if near(tg.Range, 20, 3) && near(tg.Velocity, 1, 1) {
				found = true
			}
		}
	}
	if !found {
		t.Fatalf("target never confirmed: %+v", res)
	}
}

func TestDeterministicReplay(t *testing.T) {
	seqs := []int64{0, 1, 2, 3, 4}
	a := runFrames(t, testConfig(filepath.Join(t.TempDir(), "cp1")), seqs)
	b := runFrames(t, testConfig(filepath.Join(t.TempDir(), "cp2")), seqs)
	if len(a) != len(b) {
		t.Fatalf("result count %d vs %d", len(a), len(b))
	}
	for i := range a {
		if !resultsEqual(a[i], b[i]) {
			t.Fatalf("frame %d differs:\n%+v\n%+v", i, a[i], b[i])
		}
	}
}

func TestRestartNoDuplicateNoSkip(t *testing.T) {
	dir := t.TempDir()
	cpAll := filepath.Join(dir, "cp-all")
	cpCrash := filepath.Join(dir, "cp-crash")
	all := runFrames(t, testConfig(cpAll), []int64{0, 1, 2, 3, 4})

	// Build a genuine checkpoint reflecting the state after processing
	// seq=0 and seq=1, then start a fresh service that resumes from seq=2 and
	// must reproduce the continuous run without re-emitting seq 0/1 or
	// skipping any later frame.
	runFrames(t, testConfig(cpCrash), []int64{0, 1})
	data := mustRead(t, cpCrash)
	var rec checkpointRecord
	if err := json.Unmarshal(data, &rec); err != nil {
		t.Fatal(err)
	}
	if rec.Seq != 1 || len(rec.Tracker) == 0 {
		t.Fatalf("bad seeded checkpoint: %+v", rec)
	}
	part := runFrames(t, testConfig(cpCrash), []int64{2, 3, 4})
	var want []FrameResult
	for _, r := range all {
		if r.Seq >= 2 {
			want = append(want, r)
		}
	}
	if len(part) != len(want) {
		t.Fatalf("after restart got %d results, want %d", len(part), len(want))
	}
	for i := range want {
		if !resultsEqual(part[i], want[i]) || part[i].Seq != want[i].Seq {
			t.Fatalf("restart mismatch at %d:\n%+v\n%+v", i, part[i], want[i])
		}
	}
}

func TestDuplicateDeliveryNotEmittedTwice(t *testing.T) {
	cpPath := filepath.Join(t.TempDir(), "cp")
	// First run commits seq 0..2.
	first := runFrames(t, testConfig(cpPath), []int64{0, 1, 2})
	if len(first) != 3 {
		t.Fatalf("want 3, got %d", len(first))
	}
	// A duplicate retry of seq 0..2 must emit nothing and not advance.
	again := runFrames(t, testConfig(cpPath), []int64{0, 1, 2})
	if len(again) != 0 {
		t.Fatalf("committed frames re-emitted: %+v", again)
	}
	// And the next fresh frame resumes normally.
	next := runFrames(t, testConfig(cpPath), []int64{3})
	if len(next) != 1 || next[0].Seq != 3 {
		t.Fatalf("resume after duplicate failed: %+v", next)
	}
}

func TestGapRejected(t *testing.T) {
	cfg := testConfig(filepath.Join(t.TempDir(), "cp"))
	svc, err := New(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()
	g := simGen(t)
	raw, _ := g.EncodedFrame(0)
	raw2, _ := g.EncodedFrame(2) // missing seq 1
	in := make(chan []byte, 2)
	out := make(chan FrameResult, 2)
	in <- raw
	in <- raw2
	close(in)
	if err := svc.Run(context.Background(), in, out); err != ErrFrameGap {
		t.Fatalf("want ErrFrameGap, got %v", err)
	}
}

func TestCheckpointAtomicFile(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cp")
	res := runFrames(t, testConfig(path), []int64{0, 1})
	if len(res) != 2 {
		t.Fatalf("want 2 results")
	}
	data := mustRead(t, path)
	var rec checkpointRecord
	if err := json.Unmarshal(data, &rec); err != nil {
		t.Fatalf("checkpoint not valid json: %v (%q)", err, data)
	}
	if rec.Seq != 1 {
		t.Fatalf("checkpoint seq = %d, want 1", rec.Seq)
	}
}

func TestCorruptCheckpointRejected(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cp")
	if err := os.WriteFile(path, []byte("{not-json"), 0o600); err != nil {
		t.Fatal(err)
	}
	if _, err := New(testConfig(path)); err == nil {
		t.Fatal("corrupt checkpoint must fail to load")
	}
}

func TestMemoryDoesNotGrowWithFrames(t *testing.T) {
	collect := func() uint64 {
		runtime.GC()
		var m runtime.MemStats
		runtime.ReadMemStats(&m)
		return m.HeapAlloc
	}

	process := func(t *testing.T, n int) {
		svc, err := New(testConfig(""))
		if err != nil {
			t.Fatal(err)
		}
		defer svc.Close()
		g := simGen(t)
		in := make(chan []byte, n)
		out := make(chan FrameResult, n)
		done := make(chan error, 1)
		go func() { done <- svc.Run(context.Background(), in, out) }()
		for s := 0; s < n; s++ {
			raw, _ := g.EncodedFrame(int64(s))
			in <- raw
			<-out
		}
		close(in)
		<-done
	}

	process(t, 40)
	runtime.GC()
	time.Sleep(20 * time.Millisecond)
	base := collect()
	process(t, 200)
	after := collect()
	// Allow generous slack for allocator noise; the key requirement is no
	// linear growth with processed frame count.
	if after > base*3+(4<<20) {
		t.Fatalf("heap grew from %d to %d across many frames", base, after)
	}
}

func near(a, b, tol float64) bool {
	d := a - b
	if d < 0 {
		d = -d
	}
	return d <= tol
}

func resultsEqual(a, b FrameResult) bool {
	if a.Seq != b.Seq || len(a.Targets) != len(b.Targets) || len(a.Peaks) != len(b.Peaks) {
		return false
	}
	for i := range a.Targets {
		x, y := a.Targets[i], b.Targets[i]
		if x.ID != y.ID || !near(x.Range, y.Range, 1e-9) ||
			!near(x.Velocity, y.Velocity, 1e-9) ||
			!near(x.Amplitude, y.Amplitude, 1e-9) ||
			!near(x.SNRDB, y.SNRDB, 1e-9) {
			return false
		}
	}
	for i := range a.Peaks {
		x, y := a.Peaks[i], b.Peaks[i]
		if x.RangeBin != y.RangeBin || x.DopplerBin != y.DopplerBin ||
			!near(x.Range, y.Range, 1e-9) || !near(x.Velocity, y.Velocity, 1e-9) ||
			!near(x.Amplitude, y.Amplitude, 1e-9) || !near(x.SNRDB, y.SNRDB, 1e-9) {
			return false
		}
	}
	return true
}
