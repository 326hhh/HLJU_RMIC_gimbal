#include "gimbal/transport/itransport.hpp"

#include <chrono>

namespace gimbal {

namespace {
int64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

void ITransport::send(const std::vector<uint8_t>& frame, bool urgent) {
    if (txq_.size() >= txq_max_) {
        txq_.pop_front();
        ++stats_.tx_dropped;
    }
    // urgent = 插队到最前，**不是**清空队列。
    // 一根总线常挂多台电机，清空会把别的电机刚排进去的命令一起误杀。
    if (urgent) {
        txq_.push_front(frame);
    } else {
        txq_.push_back(frame);
    }
}

bool ITransport::popFrame(zdt::Frame& out) {
    if (rxq_.empty()) return false;
    out = std::move(rxq_.front());
    rxq_.pop_front();
    return true;
}

bool ITransport::popFrameFor(uint8_t addr, zdt::Frame& out) {
    for (auto it = rxq_.begin(); it != rxq_.end(); ++it) {
        if (it->addr == addr || it->addr == zdt::ADDR_BROADCAST) {
            out = std::move(*it);
            rxq_.erase(it);
            return true;
        }
    }
    return false;
}

void ITransport::feedRx(const uint8_t* p, size_t n) {
    stats_.rx_bytes += n;
    parser_.feed(p, n);
    zdt::Frame f;
    while (parser_.pop(f)) {
        ++stats_.rx_frames;
        rxq_.push_back(std::move(f));
    }
    // 下游若不消费，避免无限增长
    while (rxq_.size() > 256) rxq_.pop_front();
}

bool ITransport::txReady() const { return (nowUs() - last_tx_us_) >= gap_us_; }

bool ITransport::nextTx(std::vector<uint8_t>& out) {
    if (txq_.empty() || !txReady()) return false;
    out = std::move(txq_.front());
    txq_.pop_front();
    last_tx_us_ = nowUs();
    return true;
}

void ITransport::onTxDone() { ++stats_.tx_frames; }

void ITransport::onTxFailed(const std::string& why) {
    stats_.last_error = why;
    stats_.connected  = false;
}

}  // namespace gimbal
