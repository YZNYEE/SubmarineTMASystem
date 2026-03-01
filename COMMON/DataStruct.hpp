struct ObsData{
    int timetamp;//时间戳
    double x;
    double y;
    double range;
    bool rngvalid{false};
    double bearing;// 单位：弧度，正北为0，顺时针增加
    bool brgvalid{false};
    double freq;
    bool freqvalid{false};
};

struct TargetState {
    double x;   // 初始位置 x
    double y;   // 初始位置 y
    double vx;  // 速度 vx
    double vy;  // 速度 vy
};