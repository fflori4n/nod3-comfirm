#include "cron_scheduler.h"

time_t Cron_Scheduler::arm_ms_before{30000};

timed_job_t Cron_Scheduler::timed_job_queue[]{};
cron_job_t Cron_Scheduler::cron_job_queue[]{};

void Cron_Scheduler::print_job(cron_job_t& job){
    ESP_LOGI("C_JOB_DBG", "");
}

esp_err_t Cron_Scheduler::compute_next_exec_time(cron_job_t& job){

    ESP_LOGI("SCHEDULER", "Computing NextRTC of job");
    return ESP_OK;
}

/*uint16_t Cron_Scheduler::generate_job_hash(int8_t eventType, uint32_t startTime, uint32_t endTime) {

      uint16_t hash = 6969;

      hash ^= eventType;
      hash ^= (startTime & 0x0000FFFF);
      hash ^= (startTime > 16);
      hash ^= (endTime & 0x0000FFFF);
      hash ^= (endTime > 16);

      return hash;
    }*/

// timed_job_t* Cron_Scheduler::add_to_job_queue(timed_job_t new_job){

//     if (cron_job_t* cron_job = dynamic_cast<cron_job_t*>(&new_job)) 
//     {
//         for(uint8_t i=0; i < 10; i++){
//             if(job_status_t::job_slot_empty != Cron_Scheduler::cron_job_queue[i].status){
//                 Cron_Scheduler::cron_job_queue[i] = *cron_job;
//                 return &(Cron_Scheduler::timed_job_queue[i]);
//             }
//         }

//         return nullptr;
//     }
//     else{

//         for(uint8_t i=0; i < 10; i++){
//             if(job_status_t::job_slot_empty != Cron_Scheduler::timed_job_queue[i].status){
//                 Cron_Scheduler::timed_job_queue[i] = new_job;
//                 return &(Cron_Scheduler::timed_job_queue[i]);
//             }
//         }

//         return nullptr;
//     }
// }

bool Cron_Scheduler::is_ready_to_execute(timed_job_t& job, time_t& unix_time){

    /* TODO: check if RTC time is correct */
    switch(job.status){

        /* if job is empty, executed, or timeslot was missed or if task is somehow unprocessed exit*/
        case job_status_t::job_slot_empty:
        case job_status_t::job_error:
        case job_status_t::job_not_executed:
        case job_status_t::job_executed:
        {
            return false;
        }
        case job_status_t::job_unprocessed:
        {
            if(ESP_OK != (Ntp_time::evaluate_rtc_time_validity(unix_time))){
                return false;
            }

            /* IF it is defined as a cron style job, first generate the next Unix time when job has to be executed.*/
            if (cron_job_t* cronJob = dynamic_cast<cron_job_t*>(&job)) 
            {
                job.next_exec_unix_time = unix_time + 60;
            /* generate next exec time and set status to unprocessed */
            } 

            /* we have RTC time. */

            if((job.next_exec_unix_time + job.timeout_seconds < unix_time) || (ESP_OK != (Ntp_time::evaluate_rtc_time_validity(job.next_exec_unix_time)))){
                job.status = job_status_t::job_not_executed;
            }
            else if((job.next_exec_unix_time + job.timeout_seconds + (Cron_Scheduler::arm_ms_before/1000) < unix_time)){
                job.status = job_status_t::job_armed;
            }

        }

        default:
        {

        }
    }
    
    job.print();
    /* TODO: write this crap*/
    // if((cjob_status_t::cjob_empty == job.job_status) ||
    // (cjob_status_t::cjob_not_executable == job.job_status) ||
    // (cjob_status_t::cjob_status_unknown == job.job_status)){
    //     return false;
    // }

    // if((cjob_status_t::cjob_returned_ok == job.job_status) ||
    // (cjob_status_t::cjob_returned_er == job.job_status) ||
    // (cjob_status_t::cjob_timeout == job.job_status) ||
    // (cjob_status_t::cjob_not_executed == job.job_status)){
    //     /* TODO: log that task was executed.*/

    //     job.job_status = cjob_status_t::cjob_unprocessed;   /* clear job, and schedule next event to be recomputed based on cron str.*/
    //     job.next_exec_unix_time = 0;
    // }

    // if((cjob_status_t::cjob_unprocessed == job.job_status) && ((time_t)0 == job.next_exec_unix_time)){
    //     if(ESP_OK == (Ntp_time::rtc_time_check())){
    //         /* process this job to get next RTC at to be executed*/
    //         printf("%lld",job.next_exec_unix_time);
    //         if(ESP_OK != Cron_Scheduler::compute_next_exec_time(job)){
    //             return false;
    //         }
    //     }
    //     else{
    //         return false;
    //     }
    // }


    /* job is active, check if time to execute.*/

    return false;
}