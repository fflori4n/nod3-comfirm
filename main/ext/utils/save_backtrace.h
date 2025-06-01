/* Based on the blogpost: https://kotyara12.ru/iot/remote_esp32_backtrace/ and https://kotyara12.ru/iot/platformio-addr2name/*/
/* And info from this guy: https://techblog.hacomono.jp/entry/2023/06/13/110000*/
// #define CONFIG_RESTART_DEBUG_STACK_DEPTH 28

// #include "debug_helpers.h"

// /*#include "/home/ffsk/esp/esp-idf/components/xtensa/include/esp_cpu_utils.h"*/


// typedef struct {  
//   size_t heap_total;
//   size_t heap_free;
//   size_t heap_free_min;
//   time_t heap_min_time;
//   #if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0
//   uint32_t backtrace [ CONFIG_RESTART_DEBUG_STACK_DEPTH ] ;
//   #endif /* CONFIG_RESTART_DEBUG_STACK_DEPTH */
// } re_restart_debug_t;

// RTC_NOINIT_ATTR re_restart_debug_t debug_info;

// void IRAM_ATTR log_backtrace_to_rtcmem(void)
// {
//     esp_backtrace_frame_t trace_pointers_obj;
//     bool is_trace_corrupted = false;

//     esp_backtrace_get_start(&(trace_pointers_obj.pc), &(trace_pointers_obj.sp), &(trace_pointers_obj.next_pc));

//     debug_info.backtrace[0] = esp_cpu_process_stack_pc(trace_pointers_obj.pc);

    // if((false == esp_stack_ptr_is_sane(trace_pointers_obj.sp)) || ( false == esp_ptr_executable((void*)esp_cpu_process_stack_pc(trace_pointers_obj.pc))))
    // {
    //     is_trace_corrupted = true;
    // }

    // uint8_t i = CONFIG_RESTART_DEBUG_STACK_DEPTH;
    // while ( i-- > 0 && trace_pointers_obj. next_pc != 0 && !is_trace_corrupted ) {  
    //   if (false == esp_backtrace_get_next_frame(&trace_pointers_obj))
    //   {  
    //     is_trace_corrupted = true;
    //   }

    //   debug_info.backtrace[CONFIG_RESTART_DEBUG_STACK_DEPTH - i] = esp_cpu_process_stack_pc(trace_pointers_obj.pc);
    // };
}

// void  __wrap_panic_print_backtrace ( const  void * f, int core) {
//     XtExcFrame* xt_frame = (XtExcFrame*) f;
//     esp_backtrace_frame_t stk_frame = { .pc = xt_frame->pc, .sp = xt_frame->a1, .next_pc = xt_frame->a0, .exc_frame = xt_frame };
//     esp_backtrace_get_start (&(stk_frame.pc), &(stk_frame.sp), &(stk_frame.next_pc));

//     // Is the first stack frame valid? 
//     bool corrupted = !( esp_stack_ptr_is_sane (stk_frame.sp) &&
//                        ( esp_ptr_executable (( void *) esp_cpu_process_stack_pc (stk_frame.pc)) ||
//                          /* Ignore the first corrupted PC in case of InstrFetchProhibited */
//                         (stk_frame.exc_frame && ((XtExcFrame*) stk_frame.exc_frame)->exccause == EXCCAUSE_INSTR_PROHIBITED)));

//     // Get the last call stack in order. 
//     uint32_t i = STACK_DEPTH;
//      while (i-- > 0 && stk_frame.next_pc != 0 && !corrupted) {
//          if (! esp_backtrace_get_next_frame (&stk_frame)) { // Get the previous stack frame. 
//             corrupted = true ;
//         }
//         s_exception_info.pc[CALL_STACK_DEPTH - i] = esp_cpu_process_stack_pc (stk_frame.pc);
//         s_exception_info.sp[CALL_STACK_DEPTH - i] = stk_frame.sp;
//     }

//     // Call the original panic_handler function to finish handling this error. 
//     __real_panic_print_backtrace (f, core);
// }

