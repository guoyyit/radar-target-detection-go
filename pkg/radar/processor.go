package radar

import (
	"context"
	"errors"
)

// Config bundles the full processing-chain configuration.
type Config struct {
	Parse    ParseConfig
	Spectrum SpectrumConfig
	CFAR     CFARConfig
	Cluster  ClusterConfig
	Tracker  TrackerConfig
}

// FrameResult is the complete output for one processed frame.
type FrameResult struct {
	FrameID    int64
	Detections []Detection
	Targets    []Target
	Rejected   bool
	Reason     ParseErrorKind
}

// ErrFrameStale indicates a frame with ID <= last checkpointed ID was skipped.
var ErrFrameStale = errors.New("frame already processed (stale)")

// ErrFrameOutOfOrder indicates a frame ID gap or backwards jump that would
// make exactly-once ordering impossible.
var ErrFrameOutOfOrder = errors.New("frame id out of order")

// Processor runs the chain with checkpointed, resumable progress.
type Processor struct {
	cfg       Config
	store     CheckpointStore
	tracker   *Tracker
	completed int64
}

// NewProcessor builds a processor and loads the persisted progress.
func NewProcessor(cfg Config, store CheckpointStore) (*Processor, error) {
	prog, err := store.Load()
	if err != nil {
		return nil, err
	}
	return &Processor{
		cfg:       cfg,
		store:     store,
		tracker:   NewTracker(cfg.Tracker),
		completed: prog.LastCompletedFrame,
	}, nil
}

// LastCompletedFrame returns the checkpoint position.
func (p *Processor) LastCompletedFrame() int64 {
	return p.completed
}

// ProcessFrame handles one raw frame end to end. The checkpoint advances only
// after the full frame result (including the tracker update) is computed, and
// is persisted before the result is returned, so a restart never duplicates or
// skips output. Rejected frames also advance the checkpoint.
func (p *Processor) ProcessFrame(raw RawFrame) (*FrameResult, error) {
	if raw.ID <= p.completed {
		return nil, ErrFrameStale
	}
	if p.completed != 0 && raw.ID != p.completed+1 {
		return nil, ErrFrameOutOfOrder
	}

	result := &FrameResult{FrameID: raw.ID}
	frame, err := ParseFrame(raw, p.cfg.Parse)
	if err != nil {
		if pe, ok := err.(*ParseError); ok {
			result.Rejected = true
			result.Reason = pe.Kind
		} else {
			return nil, err
		}
	} else {
		rd, err := ComputeRangeDoppler(frame, p.cfg.Spectrum)
		if err != nil {
			return nil, err
		}
		dets := Detect(rd, p.cfg.CFAR)
		dets = Cluster(dets, rd, p.cfg.Cluster)
		result.Detections = dets
		result.Targets = p.tracker.Update(raw.ID, dets)
	}

	if err := p.store.Save(Progress{LastCompletedFrame: raw.ID}); err != nil {
		return nil, err
	}
	p.completed = raw.ID
	return result, nil
}

// Run consumes frames from the pipe until EOF, invoking emit per completed
// frame. Stale frames (already checkpointed) are skipped; out-of-order frames
// abort with an error.
func (p *Processor) Run(ctx context.Context, pipe *Pipe, emit func(FrameResult)) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}
		raw, ok := pipe.Recv()
		if !ok {
			return nil
		}
		if raw.ID <= p.completed {
			continue
		}
		result, err := p.ProcessFrame(raw)
		if err != nil {
			if errors.Is(err, ErrFrameStale) {
				continue
			}
			return err
		}
		emit(*result)
	}
}
