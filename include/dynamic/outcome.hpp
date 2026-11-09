#pragma once

namespace tdr::dynamic {

// DM0 §6.3(6): every generated request ends in exactly one of these.
enum class Outcome { OutOfScan, Expired, Stale, RejectedNoInsertion, RejectedPromise, Accepted };

inline const char* outcome_name(Outcome o) {
    switch (o) {
        case Outcome::OutOfScan: return "out-of-scan";
        case Outcome::Expired: return "expired";
        case Outcome::Stale: return "stale";
        case Outcome::RejectedNoInsertion: return "rejected-no-insertion";
        case Outcome::RejectedPromise: return "rejected-promise";
        case Outcome::Accepted: return "accepted";
    }
    return "?";
}

} // namespace tdr::dynamic
