// 主机跑测入口: 调用被改名成 onboard_test_main() 的上板测试程序
#include <stdio.h>
#include "dlx_delay.hpp"

int onboard_test_main();

int main()
{
    try {
        onboard_test_main();
        printf("\n[runner] 测试函数自己返回了(不该发生)\n");
    } catch (OnboardRunDone &) {
        printf("\n[runner] 已跑到测试收尾的死循环, 正常结束\n");
    } catch (...) {
        printf("\n[runner] 异常退出\n");
    }
    return 0;
}

void DLX_NVIC_AutoConfig() {}
