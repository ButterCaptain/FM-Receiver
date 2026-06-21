/**
 * fm_receiver.cpp — RTL-SDR FM Receiver (C++ rewrite)
 *
 * Tunes to 98.5 MHz FM and plays audio through the default soundcard.
 *
 * Signal chain:
 *   RTL-SDR USB → Digital Mixer → FIR Decimator (÷10)
 *   → FM Discriminator → IIR Anti-alias → FIR Decimator (÷5)
 *   → De-emphasis → PortAudio output
 *
 * Sample rates: 2.4 MSPS → 240 kSPS → 48 kSPS (audio out)
 *
 * Build:
 *   g++ -std=c++17 -O2 fm_receiver.cpp \
 *       -I/opt/homebrew/include -L/opt/homebrew/lib \
 *       -lrtlsdr -lportaudio -o fm_receiver
 *
 * Usage: ./fm_receiver   (Ctrl+C to stop)
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <rtl-sdr.h>
#include <portaudio.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <csignal>

// ---- config ----

static constexpr uint32_t SDR_SAMPLE_RATE   = 2'400'000;
static constexpr uint32_t RF_DECIMATION     = 10;         // 2.4 MSPS → 240 kSPS
static constexpr uint32_t AUDIO_DECIMATION  = 5;          // 240 kSPS → 48 kSPS
static constexpr uint32_t AUDIO_SAMPLE_RATE = 48'000;

// Tune 250 kHz below the target so the DC spike from the RTL-SDR doesn't
// land on the station. We shift back down in software in the mixer stage.
static constexpr uint32_t CENTER_FREQ_HZ  = 98'250'000;
static constexpr float    MIXER_OFFSET_HZ = -250'000.0f;

static constexpr int    USB_READ_BYTES  = 512'000;
static constexpr size_t MAX_QUEUE_DEPTH = 4; // drop old buffers rather than let audio lag


//types

// normalized IQ sample, each component in [-1.0, 1.0]
struct ComplexFloat {
    float i = 0.0f;
    float q = 0.0f;
};


// RTL-SDR sends raw bytes where 128 = zero (unsigned bias)
// this just converts that to proper signed floats
static std::vector<ComplexFloat>
convert_raw_to_complex(const uint8_t* buf, size_t num_bytes)
{
    const size_t num_samples = num_bytes / 2;
    std::vector<ComplexFloat> samples(num_samples);

    for (size_t n = 0; n < num_samples; ++n) {
        samples[n].i = (static_cast<float>(buf[2 * n])     - 128.0f) / 128.0f;
        samples[n].q = (static_cast<float>(buf[2 * n + 1]) - 128.0f) / 128.0f;
    }
    return samples;
}


// Digital Mixer
// Multiplies the signal by e^(j*2π*offset*n/Fs) to shift 98.5 MHz down to DC.
// Keeps a running phase accumulator so the rotation is continuous across chunks.
class DigitalMixer {
public:
    DigitalMixer(float sample_rate, float offset_hz)
        : phase_(0.0f)
        , phase_step_(TWO_PI * offset_hz / sample_rate)
    {}

    void shift_frequency(std::vector<ComplexFloat>& samples)
    {
        for (auto& s : samples) {
            const float cos_t = std::cos(phase_);
            const float sin_t = std::sin(phase_);
            const float i_in  = s.i;
            const float q_in  = s.q;

            // complex multiply: (I + jQ)(cos + j*sin)
            s.i = i_in * cos_t - q_in * sin_t;
            s.q = i_in * sin_t + q_in * cos_t;

            phase_ += phase_step_;
            if      (phase_ >= TWO_PI) phase_ -= TWO_PI;
            else if (phase_ <  0.0f)   phase_ += TWO_PI;
        }
    }

private:
    static constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
    float phase_;
    float phase_step_;
};


// FirDecimator (complex)
// Low-pass FIR filter + downsample by N, operating on complex IQ samples.
// The delay line carries the tail of each chunk into the next so filtering
// is seamless across buffer boundaries.
class FirDecimator {
public:
    FirDecimator(const std::vector<float>& taps, size_t decimation_factor)
        : taps_(taps)
        , dec_factor_(decimation_factor)
        , tap_count_(taps.size())
        , delay_line_(taps.size() - 1, {0.0f, 0.0f})
    {}

    std::vector<ComplexFloat> process(const std::vector<ComplexFloat>& input)
    {
        std::vector<ComplexFloat> output;

        // prepend the saved history so the convolution doesn't see a gap
        std::vector<ComplexFloat> history = delay_line_;
        history.insert(history.end(), input.begin(), input.end());

        // convolve and pick every Nth output
        for (size_t i = 0; i + tap_count_ <= history.size(); i += dec_factor_) {
            ComplexFloat acc{};
            for (size_t j = 0; j < tap_count_; ++j) {
                acc.i += history[i + j].i * taps_[j];
                acc.q += history[i + j].q * taps_[j];
            }
            output.push_back(acc);
        }

        // save the tail for next time
        delay_line_.assign(history.end() - static_cast<ptrdiff_t>(tap_count_ - 1),
                           history.end());
        return output;
    }

private:
    std::vector<float>        taps_;
    std::vector<ComplexFloat> delay_line_;
    size_t tap_count_;
    size_t dec_factor_;
};


// ScalarFirDecimator (real)
// Same idea as FirDecimator but for real-valued audio samples.
// Used to bring 240 kSPS down to 48 kSPS after demodulation.
class ScalarFirDecimator {
public:
    ScalarFirDecimator(const std::vector<float>& taps, size_t decimation_factor)
        : taps_(taps)
        , dec_factor_(decimation_factor)
        , tap_count_(taps.size())
        , delay_line_(taps.size() - 1, 0.0f)
    {}

    std::vector<float> process(const std::vector<float>& input)
    {
        std::vector<float> output;

        std::vector<float> history = delay_line_;
        history.insert(history.end(), input.begin(), input.end());

        for (size_t i = 0; i + tap_count_ <= history.size(); i += dec_factor_) {
            float acc = 0.0f;
            for (size_t j = 0; j < tap_count_; ++j)
                acc += history[i + j] * taps_[j];
            output.push_back(acc);
        }

        delay_line_.assign(history.end() - static_cast<ptrdiff_t>(tap_count_ - 1),
                           history.end());
        return output;
    }

private:
    std::vector<float> taps_;
    std::vector<float> delay_line_;
    size_t tap_count_;
    size_t dec_factor_;
};


// FmDiscriminator
// Atan2 FM demodulator — measures instantaneous phase change between samples.
// Equivalent to: audio[n] = angle( x[n] * conj(x[n-1]) )
//
// Stores the last sample across calls so there's no discontinuity at
// chunk boundaries (this was the source of the popping bug in the Python version).
class FmDiscriminator {
public:
    FmDiscriminator() : last_sample_({0.0f, 0.0f}) {}

    std::vector<float> process(const std::vector<ComplexFloat>& input)
    {
        std::vector<float> audio;
        audio.reserve(input.size());

        for (const auto& s : input) {
            const float real = s.i * last_sample_.i + s.q * last_sample_.q;
            const float imag = s.q * last_sample_.i - s.i * last_sample_.q;
            audio.push_back(std::atan2(imag, real));
            last_sample_ = s;
        }
        return audio;
    }

private:
    ComplexFloat last_sample_;
};


// IirFilter
// Direct-form II transposed IIR. Handles both the audio lowpass and de-emphasis.
// State is maintained internally so it stays continuous across chunks.
class IirFilter {
public:
    IirFilter(const std::vector<float>& b_coeffs, const std::vector<float>& a_coeffs)
        : b_(b_coeffs)
        , a_(a_coeffs)
        , x_hist_(b_coeffs.size(), 0.0f)
        , y_hist_(a_coeffs.size(), 0.0f)
    {}

    std::vector<float> process(const std::vector<float>& input)
    {
        std::vector<float> output;
        output.reserve(input.size());

        for (const float x : input) {
            for (int i = static_cast<int>(x_hist_.size()) - 1; i > 0; --i)
                x_hist_[i] = x_hist_[i - 1];
            x_hist_[0] = x;

            float y = 0.0f;
            for (size_t i = 0; i < b_.size(); ++i)
                y += b_[i] * x_hist_[i];

            // skip a[0] — that's just the normalizing denominator
            for (size_t i = 1; i < a_.size(); ++i)
                y -= a_[i] * y_hist_[i - 1];

            y /= a_[0];

            for (int i = static_cast<int>(y_hist_.size()) - 1; i > 0; --i)
                y_hist_[i] = y_hist_[i - 1];
            y_hist_[0] = y;

            output.push_back(y);
        }
        return output;
    }

private:
    std::vector<float> b_, a_;
    std::vector<float> x_hist_, y_hist_;
};


// BoundedThreadSafeQueue
// Thread-safe queue with a hard cap on depth. When full, the oldest item
// gets dropped instead of blocking — keeps the DSP loop processing fresh
// data even if it falls momentarily behind the USB reader.
template <typename T>
class BoundedThreadSafeQueue {
public:
    explicit BoundedThreadSafeQueue(size_t max_depth) : max_depth_(max_depth) {}

    void push(T item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.size() >= max_depth_)
            queue_.pop(); // drop oldest rather than stall
        queue_.push(std::move(item));
        cv_.notify_one();
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return !queue_.empty(); });
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

private:
    std::queue<T>           queue_;
    std::mutex              mtx_;
    std::condition_variable cv_;
    size_t                  max_depth_;
};


static BoundedThreadSafeQueue<std::vector<uint8_t>> rf_bucket(MAX_QUEUE_DEPTH);
static std::atomic<bool> running(true);

static void signal_handler(int) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
    rf_bucket.push({}); // wake the DSP loop so it can exit cleanly
}


// sdr worker 
// Runs on a dedicated thread. Just reads USB data and drops it in the queue.
// Kept separate so the USB read never has to wait on DSP math.
// rtlsdr_read_sync blocks until a full buffer is ready, which is fine here
// since this thread has nothing else to do.
static void sdr_worker(rtlsdr_dev_t* dev) {
    while (running) {
        std::vector<uint8_t> buffer(USB_READ_BYTES);
        int bytes_read = 0;

        if (rtlsdr_read_sync(dev, buffer.data(), USB_READ_BYTES, &bytes_read) < 0)
            break;

        if (bytes_read == USB_READ_BYTES && running)
            rf_bucket.push(std::move(buffer));
    }
}


// main

int main()
{
    std::signal(SIGINT, signal_handler);

    // open RTL-SDR
    rtlsdr_dev_t* dev = nullptr;
    if (rtlsdr_open(&dev, 0) < 0) {
        std::cerr << "[ERROR] Failed to open RTL-SDR device." << std::endl;
        return 1;
    }

    rtlsdr_set_sample_rate(dev, SDR_SAMPLE_RATE);
    rtlsdr_set_center_freq(dev, CENTER_FREQ_HZ);
    rtlsdr_set_tuner_gain_mode(dev, 0); // auto gain
    rtlsdr_reset_buffer(dev);

    std::cout << "[SDR]   Center : " << CENTER_FREQ_HZ / 1e6f << " MHz\n";
    std::cout << "[SDR]   Rate   : " << SDR_SAMPLE_RATE / 1e6f << " MSPS\n";
    std::cout << "[SDR]   Target : 98.5 MHz FM\n";

    // FIR/IIR coefficients
    // RF filter: 64-tap FIR, 100 kHz cutoff at 2.4 MSPS
    // Generated with scipy.signal.firwin(64, 100e3, fs=2.4e6)
    const std::vector<float> rf_fir_taps = {
         0.000004f,  0.000012f,  0.000026f,  0.000046f,  0.000069f,  0.000090f,
         0.000101f,  0.000093f,  0.000056f, -0.000017f, -0.000132f, -0.000296f,
        -0.000506f, -0.000751f, -0.001010f, -0.001252f, -0.001435f, -0.001508f,
        -0.001419f, -0.001118f, -0.000566f,  0.000268f,  0.001385f,  0.002763f,
         0.004356f,  0.006091f,  0.007869f,  0.009572f,  0.011069f,  0.012228f,
         0.012926f,  0.013067f,  0.012926f,  0.012228f,  0.011069f,  0.009572f,
         0.007869f,  0.006091f,  0.004356f,  0.002763f,  0.001385f,  0.000268f,
        -0.000566f, -0.001118f, -0.001419f, -0.001508f, -0.001435f, -0.001252f,
        -0.001010f, -0.000751f, -0.000506f, -0.000296f, -0.000132f, -0.000017f,
         0.000056f,  0.000093f,  0.000101f,  0.000090f,  0.000069f,  0.000046f,
         0.000026f,  0.000012f,  0.000004f,  0.000001f
    };

    // Audio decimation anti-alias: 64-tap FIR, 16 kHz cutoff at 240 kSPS
    // Generated with scipy.signal.firwin(64, 16e3, fs=240e3)
    const std::vector<float> audio_fir_taps = {
         0.000006f,  0.000026f,  0.000071f,  0.000151f,  0.000272f,  0.000430f,
         0.000608f,  0.000776f,  0.000886f,  0.000876f,  0.000678f,  0.000230f,
        -0.000498f, -0.001511f, -0.002782f, -0.004237f, -0.005746f, -0.007128f,
        -0.008161f, -0.008590f, -0.008161f, -0.006712f, -0.004147f,  0.000000f,
         0.005506f,  0.012299f,  0.020163f,  0.028757f,  0.037640f,  0.046283f,
         0.054107f,  0.060537f,  0.065067f,  0.067322f,  0.067322f,  0.065067f,
         0.060537f,  0.054107f,  0.046283f,  0.037640f,  0.028757f,  0.020163f,
         0.012299f,  0.005506f,  0.000000f, -0.004147f, -0.006712f, -0.008161f,
        -0.008590f, -0.008161f, -0.007128f, -0.005746f, -0.004237f, -0.002782f,
        -0.001511f, -0.000498f,  0.000230f,  0.000678f,  0.000876f,  0.000886f,
         0.000776f,  0.000608f,  0.000430f,  0.000272f
    };

    // 2nd-order Butterworth lowpass, 15 kHz at 240 kSPS
    const std::vector<float> b_audio  = { 0.009407f, 0.018815f, 0.009407f };
    const std::vector<float> a_audio  = { 1.000000f, -1.658980f, 0.714895f };

    // 75 µs de-emphasis — undoes the pre-emphasis applied at the transmitter
    const std::vector<float> b_deemph = { 0.217469f,  0.217469f };
    const std::vector<float> a_deemph = { 1.000000f, -0.565062f };

    // build the DSP pipeline
    DigitalMixer       mixer          (static_cast<float>(SDR_SAMPLE_RATE), MIXER_OFFSET_HZ);
    FirDecimator       rf_filter      (rf_fir_taps,    RF_DECIMATION);
    FmDiscriminator    discriminator;
    IirFilter          anti_alias_lpf (b_audio,   a_audio);
    ScalarFirDecimator audio_decimator(audio_fir_taps, AUDIO_DECIMATION);
    IirFilter          de_emphasis    (b_deemph,  a_deemph);

    // open PortAudio output
    PaError   pa_err;
    PaStream* pa_stream = nullptr;

    pa_err = Pa_Initialize();
    if (pa_err != paNoError) {
        std::cerr << "[PA] Init error: " << Pa_GetErrorText(pa_err) << std::endl;
        rtlsdr_close(dev);
        return 1;
    }

    pa_err = Pa_OpenDefaultStream(
        &pa_stream,
        0, 1,                         // no input, mono output
        paFloat32,
        AUDIO_SAMPLE_RATE,
        paFramesPerBufferUnspecified, // let PortAudio pick the block size
        nullptr, nullptr              // blocking write mode, no callback needed
    );

    if (pa_err != paNoError) {
        std::cerr << "[PA] Open stream error: " << Pa_GetErrorText(pa_err) << std::endl;
        Pa_Terminate();
        rtlsdr_close(dev);
        return 1;
    }

    Pa_StartStream(pa_stream);
    std::cout << "[Audio] " << AUDIO_SAMPLE_RATE << " Hz mono output started\n";

    // spin up the hardware reader thread
    std::thread hw_thread(sdr_worker, dev);
    std::cout << "[Ready] Streaming 98.5 MHz FM — press Ctrl+C to quit.\n";

    // DSP loop runs on the main thread
    while (running) {

        std::vector<uint8_t> raw = rf_bucket.pop();
        if (!running || raw.empty()) break;

        // Stage 1: raw bytes → normalized IQ  (2.4 MSPS)
        std::vector<ComplexFloat> rf = convert_raw_to_complex(raw.data(), raw.size());

        // Stage 2: frequency shift — move 98.5 MHz to DC
        mixer.shift_frequency(rf);

        // Stage 3: RF lowpass + decimate  (2.4 MSPS → 240 kSPS)
        std::vector<ComplexFloat> baseband = rf_filter.process(rf);

        // Stage 4: FM discriminator — IQ → real audio
        std::vector<float> audio = discriminator.process(baseband);

        // Stage 5: audio IIR lowpass (15 kHz)
        std::vector<float> filtered = anti_alias_lpf.process(audio);

        // Stage 6: FIR decimate  (240 kSPS → 48 kSPS)
        std::vector<float> audio_48k = audio_decimator.process(filtered);

        // Stage 7: de-emphasis (75 µs)
        std::vector<float> final_audio = de_emphasis.process(audio_48k);

        // Stage 8: gain and hard clip to avoid distortion
        for (float& s : final_audio) {
            s *= 5.0f;
            if      (s >  1.0f) s =  1.0f;
            else if (s < -1.0f) s = -1.0f;
        }

        // Stage 9: write to soundcard
        pa_err = Pa_WriteStream(pa_stream, final_audio.data(), final_audio.size());
        if (pa_err != paNoError && running)
            std::cerr << "[PA] Write warning: " << Pa_GetErrorText(pa_err) << std::endl;
    }

    // clean shutdown
    running = false;
    rtlsdr_close(dev); // unblocks rtlsdr_read_sync in the worker thread
    if (hw_thread.joinable())
        hw_thread.join();

    Pa_StopStream(pa_stream);
    Pa_CloseStream(pa_stream);
    Pa_Terminate();

    std::cout << "[Done] Hardware released cleanly." << std::endl;
    return 0;
}
