#include <Arduino.h>

// 在这里放置函数声明：
int myFunction(int, int);

void setup() {
// 在此处放置您的设置代码，仅运行一次：
  int result = myFunction(2, 3);
}

void loop() {
// 在此处放置您的主体代码，以重复运行：
}

// 在此处放置函数定义：
int myFunction(int x, int y) {
  return x + y;
}