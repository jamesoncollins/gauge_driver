
#include <stdint.h>

class BTBuffer
{

protected:
  BTBuffer ( IRQn_Type *irqs, int num_irqs );
  static BTBuffer *BTBuffer_;

public:
  static const int dataLen = 16;
  static const int numBuffers = 32;
  BTBuffer (BTBuffer &other) = delete;
  void operator= (const BTBuffer&) = delete;
  static BTBuffer* GetInstance ();
  static void CreateInstance( IRQn_Type *irqs, int num_irqs );
  static bool pushBuffer(const uint8_t *data);
  static bool popBuffer();

private:
  /*
   * we're going to use a waste slot in order to avoid needing
   * to lock access to the full flag.
   * https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
   */
  uint8_t buffer[numBuffers][dataLen];
  int head = 0; // read from
  int tail = 0; // write to
  bool isEmpty(), isFull();

  static const int maxIRQs = 16;
  IRQn_Type irqList[maxIRQs];
  int numIRQ = 0;

  void disableIRQs();
  void enableIRQs();

};



