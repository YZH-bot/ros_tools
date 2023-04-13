#include <iostream>
#include <ncurses.h>
#include <chrono>
#include <thread>
int main()
{
    // 初始化 ncurses 库
    initscr();
    // 不显示用户输入的字符
    noecho();
    // 设置输入为非阻塞模式
    nodelay(stdscr, TRUE);

    // 设置一个循环
    bool running = true;
    while (running)
    {
        // 检查是否有键盘输入
        int ch = getch();
        if (ch != ERR)
        { // 如果有输入
            if (ch == 'q')
            { // 如果输入是 'q'，则退出循环
                running = false;
            }
            else
            { // 如果输入是其他字符，则输出该字符
                std::cout << "Input: " << ch << std::endl;
            }
        }
        std::cout << "Running: " << running << std::endl;
        // 进行其他操作，这里用一个简单的等待时间来模拟
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // 退出 ncurses 库
    endwin();

    return 0;
}
