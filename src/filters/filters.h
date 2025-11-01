#pragma once

struct DCRemover {
    explicit DCRemover(float a = 0.01f) : alpha(a), mean(0.0f) {}
    float alpha;
    float mean;
    float apply(float x) {
        mean = (1.0f - alpha) * mean + alpha * x;
        return x - mean;
    }
};

struct OnePoleLPF {
    explicit OnePoleLPF(float a = 0.0f) : alpha(a), y(0.0f), primed(false) {}
    float alpha;
    float y;
    bool  primed;
    float apply(float x) {
        if (!primed) { y = x; primed = true; return x; }
        y += alpha * (x - y);
        return y;
    }
};
