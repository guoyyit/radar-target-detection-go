// Package pipeline wires frame parsing, preprocessing, range-Doppler
// processing, CFAR detection, clustering, tracking and checkpointed frame
// progression into a local service driven by an in-memory byte channel.
package pipeline

import (
	"bytes"
	"context"

	"github.com/guoyyit/radar-target-detection-go/pkg/detect"
	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
	"github.com/guoyyit/radar-target-detection-go/pkg/signal"
	"github.com/guoyyit/radar-target-detection-go/pkg/track"
)

// FrameResult is the complete, atomic output for one frame.
type FrameResult struct {
	Seq     int64
	Peaks   []detect.Peak
	Targets []track.Target
}

// Config is the full processing configuration.
type Config struct {
	Parse          frame.ParseConfig
	Signal         signal.Config
	CFAR           detect.CFARConfig
	Cluster        detect.ClusterConfig
	Track          track.Config
	CheckpointPath string // empty disables durable checkpointing (in-memory)
}

// Service is a single-consumer processing service.
type Service struct {
	cfg   Config
	store CheckpointStore
	track *track.Tracker
	done  int64
}

// New constructs a service and restores progress from the checkpoint.
func New(cfg Config) (*Service, error) {
	var store CheckpointStore
	if cfg.CheckpointPath == "" {
		store = NewMemoryStore()
	} else {
		var err error
		store, err = NewFileStore(cfg.CheckpointPath)
		if err != nil {
			return nil, err
		}
	}
	lastSeq, err := store.Load()
	if err != nil {
		return nil, err
	}
	var last int64 = -1
	var tracker *track.Tracker
	if lastSeq != nil {
		last = lastSeq.Seq
		if len(lastSeq.Tracker) > 0 {
			tracker, err = track.RestoreState(lastSeq.Tracker)
			if err != nil {
				return nil, err
			}
		}
	}
	if tracker == nil {
		tracker = track.NewTracker(cfg.Track)
	}
	return &Service{
		cfg:   cfg,
		store: store,
		track: tracker,
		done:  last,
	}, nil
}

// Run consumes self-delimiting encoded frames from in until the channel is
// closed or ctx is canceled, emitting one FrameResult per newly completed
// frame. A frame is checkpointed only after its result has been delivered,
// and no frame state is retained after that, so memory does not grow with the
// number of processed frames.
func (s *Service) Run(ctx context.Context, in <-chan []byte, out chan<- FrameResult) error {
	defer close(out)
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case raw, ok := <-in:
			if !ok {
				return nil
			}
			f, err := frame.Parse(bytes.NewReader(raw), s.cfg.Parse)
			if err != nil {
				return err
			}
			if f.Seq <= s.done {
				// Replayed frame after restart: do not process or emit twice.
				continue
			}
			if f.Seq != s.done+1 {
				return ErrFrameGap
			}

			iq := make([][]complex128, f.Header.PulseCount)
			for p := 0; p < f.Header.PulseCount; p++ {
				iq[p] = make([]complex128, f.Header.SamplesPerPulse)
				for n := 0; n < f.Header.SamplesPerPulse; n++ {
					iq[p][n] = complex(f.I[p][n], f.Q[p][n])
				}
			}
			spec, err := signal.RangeDoppler(iq, s.cfg.Signal)
			if err != nil {
				return err
			}
			mask, floor := detect.Detect(spec, s.cfg.CFAR)
			peaks := detect.Cluster(spec, mask, floor, f.Header, s.cfg.Cluster)
			targets := s.track.Update(f.Seq, peaks)

			res := FrameResult{
				Seq:     f.Seq,
				Peaks:   peaks,
				Targets: targets,
			}
			select {
			case <-ctx.Done():
				return ctx.Err()
			case out <- res:
			}
			// Commit only after the output is durably handed off.
			state, err := s.track.MarshalState()
			if err != nil {
				return err
			}
			if err := s.store.Save(checkpointRecord{Seq: f.Seq, Tracker: state}); err != nil {
				return err
			}
			s.done = f.Seq
		}
	}
}

// LastCheckpoint returns the seq of the last committed frame (-1 when none).
func (s *Service) LastCheckpoint() int64 { return s.done }

// Close releases the checkpoint store.
func (s *Service) Close() error { return s.store.Close() }
