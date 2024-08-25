class Ntp_time{

    static constexpr char* envTz{"CST-8"};
    static constexpr time_t unixTimeAnnoDomini{1723930276};

    time_t unixTimeNow;
    time_t unixRTCTimeLastUpdatedAt;

    struct tm tmCurrentTime;
    char strftimeBuffer[64] = {'\0'};

    public:
        uint32_t espTimerUptime{0};

    public:
        Ntp_time(void);
        void print(void); 
        time_t getCurrentRTCTime(bool timeNonValidReturn);
        void dbgLogLocalTimeT(time_t& rtcTime);
};