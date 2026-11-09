#pragma once

// Portable, deterministic pseudo-random numbers for the static
// reproduction (M1, 9 Nov 2026).
//
// WHY NOT <random>: the distributions in the C++ standard library
// (uniform_int_distribution, uniform_real_distribution, shuffle) are
// implementation-defined, so the same seed can give different instances
// and different search trajectories on different compilers. Reproducibility
// of every experiment (Phase J/K) requires bit-identical streams on any
// platform, so the generator AND the transformations from raw bits to
// integers/reals/permutations are implemented here, from the published
// algorithms:
//   - SplitMix64 (Steele, Lea & Flood 2014; Vigna's public-domain reference),
//   - xoshiro256** 1.0 (Blackman & Vigna 2018; public domain).
// Neither is a statement about the paper: Paper 1 gives no random-number
// generator or seed policy ([ASSUMPTION], DECISIONS.md "M1").
//
// All arithmetic is unsigned 64-bit (well-defined wrap-around), and the
// double conversion uses the top 53 bits, so results are exactly
// reproducible.

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace tdr::util {

// SplitMix64: used to expand one 64-bit seed into generator state and to
// derive independent seeds.
class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        state_ += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
private:
    std::uint64_t state_;
};

// Named seed streams: each consumer of randomness gets its own stream so
// that, for example, changing the number of VNS iterations can never change
// the instance that was generated from the same base seed.
enum class SeedStream : std::uint64_t {
    Instance = 1,        // customer coordinates
    InitialSolution = 2, // k-means seeding and any randomised construction step
    Vns = 3,             // acceptance draws and neighbourhood tie-breaking
    Shake = 4,           // shake selection and parameters
    Fuzz = 5,            // test-only
};

// Derives an independent 64-bit seed from (base seed, stream, index).
// Deterministic; different (stream, index) pairs give unrelated seeds.
// `index` is a dataset number or a replication number.
inline std::uint64_t derive_seed(std::uint64_t base_seed, SeedStream stream, std::uint64_t index = 0) {
    SplitMix64 a(base_seed);
    const std::uint64_t k1 = a.next();
    SplitMix64 b(k1 ^ (static_cast<std::uint64_t>(stream) * 0xd1b54a32d192ed03ULL));
    const std::uint64_t k2 = b.next();
    SplitMix64 c(k2 ^ (index * 0x8cb92ba72f3d8dd7ULL));
    return c.next();
}

// xoshiro256** 1.0.
class Rng {
public:
    explicit Rng(std::uint64_t seed = 1) { reseed(seed); }

    void reseed(std::uint64_t seed) {
        SplitMix64 sm(seed);
        for (auto& w : s_) w = sm.next();
        // The all-zero state is the only invalid one; SplitMix64 cannot
        // produce four zero words in a row, but guard anyway.
        if ((s_[0] | s_[1] | s_[2] | s_[3]) == 0) s_[0] = 1;
    }

    std::uint64_t next_u64() {
        const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
        const std::uint64_t t = s_[1] << 17;
        s_[2] ^= s_[0];
        s_[3] ^= s_[1];
        s_[1] ^= s_[2];
        s_[0] ^= s_[3];
        s_[2] ^= t;
        s_[3] = rotl(s_[3], 45);
        return result;
    }

    // Uniform double in [0, 1): top 53 bits / 2^53.
    double uniform01() {
        return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
    }

    // Uniform double in [lo, hi).
    double uniform_real(double lo, double hi) { return lo + (hi - lo) * uniform01(); }

    // Unbiased uniform integer in [lo, hi] (inclusive), lo <= hi, by
    // rejection sampling on the 64-bit stream.
    std::int64_t uniform_int(std::int64_t lo, std::int64_t hi) {
        const std::uint64_t range = static_cast<std::uint64_t>(hi) - static_cast<std::uint64_t>(lo) + 1ULL;
        if (range == 0) return static_cast<std::int64_t>(next_u64()); // full 64-bit range
        const std::uint64_t threshold = (0ULL - range) % range;      // = 2^64 mod range
        std::uint64_t x;
        do { x = next_u64(); } while (x < threshold);
        return static_cast<std::int64_t>(static_cast<std::uint64_t>(lo) + x % range);
    }

    // Uniform index in [0, n), n >= 1.
    std::size_t uniform_index(std::size_t n) {
        return static_cast<std::size_t>(uniform_int(0, static_cast<std::int64_t>(n) - 1));
    }

    // Fisher-Yates shuffle (descending), defined here so it is identical on
    // every platform.
    template <typename T>
    void shuffle(std::vector<T>& v) {
        for (std::size_t i = v.size(); i > 1; --i) {
            const std::size_t j = uniform_index(i);
            std::swap(v[i - 1], v[j]);
        }
    }

    // true with probability p (p <= 0 never, p >= 1 always).
    bool bernoulli(double p) { return uniform01() < p; }

private:
    static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    std::uint64_t s_[4];
};

} // namespace tdr::util
