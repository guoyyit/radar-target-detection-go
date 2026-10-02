// Package track associates per-frame peaks into tracks and manages target
// confirmation, coasting (brief-loss hold), deletion and smoothing.
package track

import (
	"math"
	"sort"

	"github.com/guoyyit/radar-target-detection-go/pkg/detect"
)

// Target is one confirmed, smoothed track output.
type Target struct {
	ID        int64
	Range     float64
	Velocity  float64
	Amplitude float64
	SNRDB     float64
}

// Config configures gating, confirmation, hold and smoothing.
type Config struct {
	RangeGate    float64
	VelocityGate float64
	ConfirmCount int // consecutive associations before output
	HoldFrames   int // coasting frames after the last association
	WindowLength int // trailing sliding-window smoothing length
}

type phase int

const (
	phasePending phase = iota
	phaseConfirmed
)

type track struct {
	id        int64
	phase     phase
	hits      int // consecutive associations (confirmation window)
	misses    int // consecutive misses since the last hit
	rangeWin  []float64
	velWin    []float64
	ampWin    []float64
	snrWin    []float64
	head      int
	filled    int
	lastRange float64
	lastVel   float64
	amp       float64
	snr       float64
}

// Tracker keeps only live track state; its memory is bounded by the number of
// simultaneously tracked targets and WindowLength, never by processed frames.
type Tracker struct {
	cfg    Config
	nextID int64
	live   []*track
}

// NewTracker constructs a tracker with validated defaults.
func NewTracker(cfg Config) *Tracker {
	if cfg.ConfirmCount <= 0 {
		cfg.ConfirmCount = 1
	}
	if cfg.HoldFrames < 0 {
		cfg.HoldFrames = 0
	}
	if cfg.WindowLength <= 0 {
		cfg.WindowLength = 1
	}
	return &Tracker{cfg: cfg, nextID: 1}
}

// Update ingests one frame's peaks and returns the currently confirmed targets
// (coasted targets retain their last smoothed values).
func (t *Tracker) Update(seq int64, peaks []detect.Peak) []Target {
	used := make([]bool, len(peaks))

	// Deterministic association: existing tracks in ascending ID order, each
	// taking the nearest unused peak inside the elliptical gate.
	for _, tr := range t.live {
		best := -1
		bestCost := math.Inf(1)
		for i := range peaks {
			if used[i] {
				continue
			}
			dR := (peaks[i].Range - tr.lastRange) / t.cfg.RangeGate
			dV := (peaks[i].Velocity - tr.lastVel) / t.cfg.VelocityGate
			cost := dR*dR + dV*dV
			if cost <= 1 && cost < bestCost {
				bestCost = cost
				best = i
			}
		}
		if best >= 0 {
			used[best] = true
			t.observe(tr, peaks[best])
		} else {
			tr.misses++
			if tr.phase == phasePending {
				tr.hits = 0
			}
		}
	}

	// Unused peaks spawn pending tracks, deterministically ordered by the
	// input peak order (range, then Doppler upstream).
	for i := range peaks {
		if used[i] {
			continue
		}
		tr := &track{
			id:       t.nextID,
			hits:     0,
			misses:   0,
			rangeWin: make([]float64, t.cfg.WindowLength),
			velWin:   make([]float64, t.cfg.WindowLength),
			ampWin:   make([]float64, t.cfg.WindowLength),
			snrWin:   make([]float64, t.cfg.WindowLength),
		}
		t.nextID++
		t.observe(tr, peaks[i])
		t.live = append(t.live, tr)
	}

	survivors := t.live[:0]
	out := make([]Target, 0)
	for _, tr := range t.live {
		if tr.misses > t.cfg.HoldFrames {
			continue
		}
		survivors = append(survivors, tr)
		if tr.phase == phaseConfirmed {
			out = append(out, t.snapshot(tr))
		}
	}
	t.live = survivors

	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out
}

func (t *Tracker) observe(tr *track, p detect.Peak) {
	tr.misses = 0
	tr.hits++
	if tr.hits >= t.cfg.ConfirmCount {
		tr.phase = phaseConfirmed
	}
	tr.lastRange = p.Range
	tr.lastVel = p.Velocity
	tr.amp = p.Amplitude
	tr.snr = p.SNRDB

	if tr.rangeWin == nil {
		tr.rangeWin = make([]float64, t.cfg.WindowLength)
		tr.velWin = make([]float64, t.cfg.WindowLength)
		tr.ampWin = make([]float64, t.cfg.WindowLength)
		tr.snrWin = make([]float64, t.cfg.WindowLength)
	}
	push(tr.rangeWin, &tr.head, &tr.filled, p.Range)
	push(tr.velWin, &tr.head, &tr.filled, p.Velocity)
	push(tr.ampWin, &tr.head, &tr.filled, p.Amplitude)
	push(tr.snrWin, &tr.head, &tr.filled, p.SNRDB)
}

// push writes into a ring buffer; head points one past the newest sample.
func push(win []float64, head, filled *int, v float64) {
	win[*head] = v
	*head++
	if *head == len(win) {
		*head = 0
	}
	if *filled < len(win) {
		*filled++
	}
}

func mean(win []float64, head, filled int) float64 {
	if filled == 0 {
		return 0
	}
	var sum float64
	for i := 0; i < filled; i++ {
		sum += win[(head-filled+i+len(win))%len(win)]
	}
	return sum / float64(filled)
}

func (t *Tracker) snapshot(tr *track) Target {
	return Target{
		ID:        tr.id,
		Range:     mean(tr.rangeWin, tr.head, tr.filled),
		Velocity:  mean(tr.velWin, tr.head, tr.filled),
		Amplitude: mean(tr.ampWin, tr.head, tr.filled),
		SNRDB:     mean(tr.snrWin, tr.head, tr.filled),
	}
}

// LiveCount exposes the number of retained tracks (mainly for tests/metrics).
func (t *Tracker) LiveCount() int { return len(t.live) }
