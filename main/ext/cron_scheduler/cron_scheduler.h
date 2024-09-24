#include <esp_err.h>
#include <esp_netif_sntp.h>

enum class cjob_status_t{
    cjob_empty          = 0,
    cjob_unprocessed    = 1,
    cjob_not_executable = 2,
    cjob_waiting        = 3,
    cjob_armed          = 4,
    cjob_in_progress    = 5,
    cjob_returned_ok    = 6,
    cjob_returned_er    = 7,
    cjob_timeout        = 8,
    cjob_status_unknown = 9,
};

typedef struct cron_job_t{

    cjob_status_t job_status;
    time_t next_exec_unix_time;
    char* cron_str = "\0";
    uint32_t job_timeout_seconds;

}cron_job_t;

class Cron_Scheduler{

    private:

    esp_err_t compute_next_exec_time(void);
    public:

    bool is_ready_to_execute(cron_job_t& job);

};