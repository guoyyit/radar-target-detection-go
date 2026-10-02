package radar

import (
	"runtime"
	"testing"
)

func TestTrackerMemoryBounded(t *testing.T) {
	tr := NewTracker(TrackerConfig{
		AssocRange: 1, AssocVelocity: 1,
		ConfirmCount: 1, HoldFrames: 2, WindowSize: 3,
	})
	for frame := int64(1); frame <= 200; frame++ {
		tr.Update(frame, []Detection{det(10+float64(frame%2), 0.5)})
	}
	if len(tr.tracks) > 4 {
		t.Fatalf("track count grew with frames: %d", len(tr.tracks))
	}
}

func TestProcessorMemoryBoundedOverManyFrames(t *testing.T) {
	cfg := testConfig()
	p, err := NewProcessor(cfg, NewMemoryCheckpointStore())
	if err != nil {
		t.Fatal(err)
	}
	var m1, m2 runtime.MemStats
	for id := int64(1); id <= 100; id++ {
		if _, err := p.ProcessFrame(simFrame(id, 500+id)); err != nil {
			t.Fatal(err)
		}
	}
	runtime.GC()
	runtime.ReadMemStats(&m1)
	for id := int64(101); id <= 300; id++ {
		if _, err := p.ProcessFrame(simFrame(id, 500+id)); err != nil {
			t.Fatal(err)
		}
	}
	runtime.GC()
	runtime.ReadMemStats(&m2)
	// Heap objects must not grow proportionally with processed frame count.
	if m2.HeapObjects > m1.HeapObjects*3 {
		t.Fatalf("heap objects grew with frames: %d -> %d", m1.HeapObjects, m2.HeapObjects)
	}
}

func TestWindowAndPadding(t *testing.T) {
	frame, err := ParseFrame(validRaw(), parseCfg())
	if err != nil {
		t.Fatal(err)
	}
	pp, err := PreparePulses(frame, WindowConfig{
		RangeWindow: WindowHann, DopplerWindow: WindowHamming,
		RemoveDC: true, RangePadTo: 128, DopplerPadTo: 16,
	})
	if err != nil {
		t.Fatal(err)
	}
	if pp.RangeFFTSize != 128 || pp.DopplerSize != 16 {
		t.Fatalf("pad sizes = %d,%d want 128,16", pp.RangeFFTSize, pp.DopplerSize)
	}
	// Hann endpoints are zero.
	if pp.Re[0][0] != 0 || pp.Im[0][0] != 0 {
		t.Fatalf("zero-padded/Hann end sample not zero: %v", pp.Re[0][0])
	}
	// Zero padding tail is zero.
	if pp.Re[0][64] != 0 || pp.Re[0][127] != 0 {
		t.Fatal("zero padding tail must be zero")
	}
}
