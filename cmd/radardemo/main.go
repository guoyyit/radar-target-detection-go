// radardemo drives the local FMCW processing service with frames produced on
// an in-memory channel by the deterministic simulator, then prints detected
// targets per frame.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"

	"github.com/guoyyit/radar-target-detection-go/pkg/detect"
	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
	"github.com/guoyyit/radar-target-detection-go/pkg/pipeline"
	"github.com/guoyyit/radar-target-detection-go/pkg/signal"
	"github.com/guoyyit/radar-target-detection-go/pkg/sim"
	"github.com/guoyyit/radar-target-detection-go/pkg/track"
)

func main() {
	frames := flag.Int("frames", 12, "number of frames to process")
	cpPath := flag.String("checkpoint", "", "checkpoint file path (empty = memory only)")
	seed := flag.Int64("seed", 1234, "simulator random seed")
	flag.Parse()

	h := frame.Header{
		SampleRate:      10e6,
		ChirpSlope:      12e12,
		CarrierFreq:     77e9,
		PulseInterval:   40e-6,
		PulseCount:      32,
		SamplesPerPulse: 128,
	}
	gen, err := sim.New(sim.Config{
		Header: h,
		Targets: []sim.TargetSpec{
			{Range: 20, Velocity: 1, Amplitude: 8},
			{Range: 40, Velocity: -2, Amplitude: 8},
		},
		NoiseSigma: 0.05,
		DCOffsetI:  3,
		Seed:       *seed,
	})
	if err != nil {
		log.Fatal(err)
	}

	cfg := pipeline.Config{
		Parse: frame.ParseConfig{},
		Signal: signal.Config{
			Window: signal.WindowHann, RemoveDC: true,
			RangeZeroPad: 2, DopplerZeroPad: 2,
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
		CheckpointPath: *cpPath,
	}
	svc, err := pipeline.New(cfg)
	if err != nil {
		log.Fatal(err)
	}
	defer svc.Close()

	in := make(chan []byte, *frames)
	out := make(chan pipeline.FrameResult, *frames)
	for s := int64(0); s < int64(*frames); s++ {
		raw, err := gen.EncodedFrame(s)
		if err != nil {
			log.Fatal(err)
		}
		in <- raw
	}
	close(in)

	go func() {
		if err := svc.Run(context.Background(), in, out); err != nil {
			log.Fatal(err)
		}
	}()

	for res := range out {
		fmt.Printf("frame %d: %d target(s)\n", res.Seq, len(res.Targets))
		for _, tg := range res.Targets {
			fmt.Printf("  id=%-3d range=%7.2fm  vel=%6.2fm/s  amp=%9.1f  snr=%5.1fdB\n",
				tg.ID, tg.Range, tg.Velocity, tg.Amplitude, tg.SNRDB)
		}
	}
	fmt.Printf("last checkpoint seq: %d\n", svc.LastCheckpoint())
}
