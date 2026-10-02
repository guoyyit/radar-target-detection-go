package track

import (
	"testing"

	"github.com/guoyyit/radar-target-detection-go/pkg/detect"
)

func peak(r, v, snr float64) detect.Peak {
	return detect.Peak{Range: r, Velocity: v, Amplitude: 10, SNRDB: snr}
}

func TestConfirmationRequiresConsecutiveFrames(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 2, VelocityGate: 2, ConfirmCount: 3, HoldFrames: 1, WindowLength: 1})
	if got := tr.Update(0, []detect.Peak{peak(10, 1, 20)}); len(got) != 0 {
		t.Fatalf("pending target must not be emitted: %+v", got)
	}
	if got := tr.Update(1, []detect.Peak{peak(10, 1, 20)}); len(got) != 0 {
		t.Fatalf("2/3 hits must not emit")
	}
	got := tr.Update(2, []detect.Peak{peak(10, 1, 20)})
	if len(got) != 1 || got[0].Range != 10 {
		t.Fatalf("3/3 hits must emit: %+v", got)
	}
}

func TestConfirmCounterResetsOnMiss(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 2, VelocityGate: 2, ConfirmCount: 3, HoldFrames: 5, WindowLength: 1})
	tr.Update(0, []detect.Peak{peak(10, 1, 20)})
	tr.Update(1, nil)
	tr.Update(2, []detect.Peak{peak(10, 1, 20)})
	if got := tr.Update(3, []detect.Peak{peak(10, 1, 20)}); len(got) != 0 {
		t.Fatalf("counter should reset after gap: %+v", got)
	}
	if got := tr.Update(4, []detect.Peak{peak(10, 1, 20)}); len(got) != 1 {
		t.Fatalf("should confirm on 3 consecutive: %+v", got)
	}
}

func TestHoldThenDelete(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 2, VelocityGate: 2, ConfirmCount: 1, HoldFrames: 2, WindowLength: 1})
	tr.Update(0, []detect.Peak{peak(10, 1, 20)})
	if got := tr.Update(1, nil); len(got) != 1 {
		t.Fatalf("coast frame 1 must retain target")
	}
	if got := tr.Update(2, nil); len(got) != 1 {
		t.Fatalf("coast frame 2 must retain target")
	}
	if got := tr.Update(3, nil); len(got) != 0 {
		t.Fatalf("target must be deleted after hold: %+v", got)
	}
	if tr.LiveCount() != 0 {
		t.Fatalf("live tracks must be pruned: %d", tr.LiveCount())
	}
}

func TestReacquireWithinHoldKeepsID(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 2, VelocityGate: 2, ConfirmCount: 1, HoldFrames: 3, WindowLength: 1})
	first := tr.Update(0, []detect.Peak{peak(10, 1, 20)})
	tr.Update(1, nil)
	again := tr.Update(2, []detect.Peak{peak(10, 1, 20)})
	if first[0].ID != again[0].ID {
		t.Fatalf("reacquired target must keep ID: %d vs %d", first[0].ID, again[0].ID)
	}
}

func TestDistinctPeaksSpawnDistinctTracks(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 2, VelocityGate: 2, ConfirmCount: 1, HoldFrames: 0, WindowLength: 1})
	got := tr.Update(0, []detect.Peak{peak(10, 0, 20), peak(50, 0, 20)})
	if len(got) != 2 || got[0].ID == got[1].ID {
		t.Fatalf("two far-apart peaks must create two tracks: %+v", got)
	}
}

func TestSmoothingWindow(t *testing.T) {
	tr := NewTracker(Config{RangeGate: 50, VelocityGate: 50, ConfirmCount: 1, HoldFrames: 0, WindowLength: 3})
	tr.Update(0, []detect.Peak{peak(10, 0, 20)})
	tr.Update(1, []detect.Peak{peak(12, 0, 20)})
	got := tr.Update(2, []detect.Peak{peak(14, 0, 20)})
	if got[0].Range != 12 {
		t.Fatalf("trailing mean of 10,12,14 must be 12, got %v", got[0].Range)
	}
	got = tr.Update(3, []detect.Peak{peak(16, 0, 20)})
	if got[0].Range != 14 {
		t.Fatalf("trailing mean of 12,14,16 must be 14, got %v", got[0].Range)
	}
}
