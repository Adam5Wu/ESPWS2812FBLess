# Framebuffer-less WS2812 driver for ESP8266 RTOS
Driving WS2812 strips using UART1 TX (GPIO 02) without buffering an entire frame.

**TL;DR** for the impatient: this project allows you to drive **over 1000 pixels** of WS2812
compatible LEDs with **less than 200 bytes** buffer. [Try out the example](#how-to-use).

## The Problem
Existing addressable LED driving solutions for ESP2812 require buffering an entire "frame" in memory.
And if you think that's 3 bytes (RGB) per pixel, you are wrong -- due to how WS2812 "perceives" data,
it usually takes 12 bytes (!) to represent a pixel.

Hence a 300 pixel strip would consume 3600 bytes to buffer a frame! On top of that, to display a
smooth continuous transition, you will need at least 2 frames -- drawing on one frame while sending
the other, and that adds up to 7200 bytes of precious RAM on a tiny device with just a total 80KB of
them (realistically you only have about 50KB available, since the OS, file system and WiFi will all
consume some memory; and even less if you are also running WebServer or other services.)

## Theory of Operation
A full frame buffer is technically not really necessary, because the processor is capable of producing
pixel data faster than the LEDs can consume.

According to the WS2812 data sheet, each bit is represented as two signal levels spanning an average
of 1.25$\mu$s (microseconds). Hence a 24-bit RGB pixel takes 30$\mu$s to transfer. ESP8266 is capable
of running at 160Mhz, which means that for every pixel transferred, the CPU could run 4800 cycles.

Let's assume 4000 instructions can be executed during that time. So as long as a pixel's data takes
less than 4000 instructions to generate, we could *theoretically* produce the pixel data on-the-fly.
*No buffering needed!* :D

### Scheduling Jitters
Of course the above analysis is very idealistic. In reality, the CPU is running multiple tasks
concurrently, so not all cycles are available for pixel data generation.

And to make matters worse, sometimes a high-priority task (such as WiFi tasks, or other interrupts)
can occupy the CPU for a prolonged period of time without releasing, thereby "starving" the pixel
generation task. Adding salt to the injury, WS2812 operates with very strict timing: a 50$\mu$s
quiescence is interpreted as a "reset". So if the pixel generation task is preempted for more than
50$\mu$s, the LEDs will interpret subsequently produced pixels as "start from the beginning" --
and that is the common cause of "flickering": pixels displayed at wrong locations.

As a result, even if each pixel only requires a few hundreds, or even less instructions to produce,
without any buffer the rendering tends to produce a lot of random defects, which is not acceptable.

So, some amount of buffering is still needed -- just not as excessive as buffering the entire frame
(when the frame is large). The amount of buffer needed can be expressed as:
`MIN(<frame size>, <scheduling jitter budget>)`, where `<scheduling jitter budget>` is the worst
preemption interval we anticipate.

For example, it is reported that on ESP8266 the worst scheduling jitter is produced by the WiFi
sub-system, with magnitudes up to 800us. Therefore, we can use a 1200$\mu$s budget to comfortably
absorb scheduling jitters. And since each WS2812 pixel takes 30$\mu$s to transfer, the scheduling
jitter budget translates to a buffer of 40 pixels. So, if the strip is under 40 pixels, we buffer
the entire frame; but for strips more than 40 pixels, regardless of 50, 300, or 1000 pixels, we
always only need a *40 pixel buffer*!

### RGB Pixel vs. UART "pixel"
WS2812 perceives pixel data bits as transitions between two levels over time. As a result data we
send over the UART line is more verbose than the original RGB data. However, that doesn't mean we
have to buffer the verbose representation. Because we have ample amount of processing power at our
disposal, we could trade that with memory space -- the data can be buffered in more compact RGB
format, and on-the-fly transcribed into the verbose representation when they are sent via UART.

So, to drive a 300 pixel strip, we only need 40 * 3 = 120 bytes buffer, as opposed to 3600 bytes.
That is a whooping 96.7% saving!

### Frame "Underflow" Handling
Finally, without having precise and complete knowledge of the entire platform (which is not possible
for ESP8266, as part of the system, such as WiFi, is closed source), there is always some possibility
for a scheduling jitter of surprising magnitude to disrupt the rendering. But we could minimize the
damage by handling the disruption in a correct way.

When we temporarily run out of pixels to send before finishing rendering an entire frame, WS2812 will
reset to the first pixel. Under such situation, trying to send the rest of the frame when scheduling
resumes will do more harm than good -- it will result in some pixel data displayed at a wrong location,
albeit very briefly, often enough to produce a very visible "flicker".

A much better solution is to simply "drop" the rest of the frame. Assuming the transition sequence
being rendered is *smooth*, that means the changes across two neighboring frames are rather small.
Therefore, even if we skip updating a region of pixels for a frame and resume updating them in the
next frame, it should not result in very noticeable visual defects. Basically, doing **nothing** is
better than doing **the wrong thing**.

## How to use
Refer to the `example` directory for a minimalistic demo.

*Caution: the stock ESP8266 RTOS SDK's `timer_struct.h` has an
[incorrect definition](https://github.com/dsptech/ESP8266_RTOS_SDK/tree/fix_frc1_plus_register_layout).
This library assumes this header was already patched.
You can get an already patched version of the SDK by using the `example/platform.ini` for your project.*

### API Documentation
The APIs are broken down into two parts: the driver, and the frame rendering.

#### Driver Interface
The driver API interface is pretty simple and self-explanatory. Notable common adjustment levers are:
1. `IOConfig::pixel_format`: You can select one of six permutations of RGB color ordering. Please refer
   to your LED's component datasheet;
2. `IOConfig::std_reset_us` and `IOConfig::min_reset_us`: You should set `std_reset_us` according to
   your LED's component datasheet; however, for `min_reset_us`, you will need to determine a good value
   by test running with the driver.

   You want to set this value as close to your LED's **lower bound** as possible, because that allows
   the driver to accurately detect [frame underflow](#frame-underflow-handling) in case it happens, and
   apply proper handling to minimize visual defects.

   Note: *Only* perform the timing test using the [custom ring buffer](#ring-buffer). Do NOT use the
   IDF ring buffer for this task. Due to its lower efficiency, it cannot produce good-looking visuals
   even if the timings are correct, which will make the timing adjustment exceedingly difficult.

   * Say if you have a 200 pixel strip, configure the renderer with *less* pixels, e.g. 72 or 108,
     and a high target FPS, e.g. 100.
     - Using a multiple of 9 pixels can help you better determine a good `std_reset_us` value.
   * Set up the driver using a custom `IOConfig` with a *very small* jitter buffer, e.g.
     `CONFIG_WS2812_CUSTOM(<std_reset_us>, <min_reset_us>, 270)`.
   * Run 10~30 sec "color dot" transitions, this would create plenty of opportunities for potential frame
     underflow near-misses and actual misses.
   * Start by setting both `std_reset_us` and `min_reset_us` to the value from the datasheet:
     - Unless you are very lucky and those numbers are exactly right, you should lots of visual defects.
     - If you see *more* pixels than you configured lit up, that means `std_reset_us` is too **low**;
     - If you see flickers, that means `min_reset_us` is too **high**.
     - You may see both happening at the same time.
   * Resolve the issues one-by-one:
     - First, find a better `std_reset_us`: gradually increase its value until *only* your configured
       number of pixels light up. (You will probably need to power-off the strip in between each trial.)
       - Only proceed to the next step if your `std_reset_us` is good.
     - Then find a better `std_reset_us`: gradually reduce its value. When you are close to the good
       value, you should see the flickering begin to reduce, and eventually stops completely when you
       get to a good value.
       - Note that if you make big reductions, and the value is *too low*, the flickering disappears
         completely but another visual defect emerges: the moving dot "stutters" (i.e. pause-resume-pause).
         Increase the value to reduce the stutter, until you see a smooth transition.

5. `DriverStart()` parameters, `task_stack` and `task_priority`:
   - Pixel data are [generated dynamically](#frame-rendering-api), and this process is driven by a
     dedicated task, whose stack space is consumed during this process. If you have a complex
     transition you may run out of the default allocated stack space. When that happens you can
     customize this parameter.
   - Although the driver (with properly configured reset time, see above) will minimize visual defects
     of a frame underflow, having too many of those will still degrade the visual experience. If a
     demanding tasks is running concurrently, you can boost the rendering task's priority to avoid
     it from being starved, and producing more smooth transitions.

#### Frame Rendering API

First, some concepts:

* A "light show" is modeled as a sequence of **transitions**, and each transition composes of
a duration, and a desired "target state":

  `Initial State` --*(Duration 1)*--> `State 1` --*(Duration 2)*--> `State 2` --(...)--> `State X`

* A *transition* is basically a sequence of **frames** that splices between two states across time:

  `Start State`-->--*(Frame 1)*-->--*(Frame 2)*-->--(...)-->--*(Frame Y)*-->--`End State`

  - The number of frames is dependent on how fast a frame can be transmitted, as well as a
    configurable "target FPS".
  - *More importantly* there is an implicit parameter: the "progression factor", i.e. which point
    between the start and end state is the frame at -- that determines the content of the frame.

* And the content of a *frame* is just a sequence of color pixels, determined by the strip length.

  `(Pixel[0])--(Pixel[1])--(Pixel[2])--(...)--(Pixel[N])`

The rendering APIs reflect the aforementioned concepts:

1. `Renderer::Enqueue()` allows you to compose a "light show" by adding a sequence of "targets";
2. "Targets" are abstract concepts, which corresponds to the `Target` abstract class:
   - All targets *must* have a common concept of `duration` (in $\mu$s);
   - Two `Target` implementations are provided in the "stock" implementation:
     - `UniformColorTarget`: Displays a single color across the entire strip;
     - `ColorDotTarget`: Displays a color dot at a certain position on the strip.
   - You could implement additional targets that perform fancier transitions.
3. When the renderer runs the transition, it will:
   - Invoke `Target::RenderInit()` at the start of the transition, providing a `Frame` as the
     initial state. The target can decide whether it will let the renderer to perform a generic
     blending during the transition, or it will do custom transitioning.
     - The generic blending will "fade-out" the initial frame and "fade-in" the frame rendered
       by the current target (see next bulletin), spliced across the entire duration;
     - Custom transition can, well, do whatever they want. :P
   - Keep calling `Target::RenderFrame()`, providing the progression factor (12-bit precision)
     linear to time passed from the transition start. The `Target` implementation should produce
     a new `Frame` instance for each call.
   - Regardless of the timing in between a transition, the renderer will ensure the final frame
     of a target is rendered (even if the time passed exceeds the target duration). And that frame
     will be provided to the next target as the initial state of the transition.
1. "Frames" are also abstract concepts, which corresponds to the `Frame` abstract class:
   - A frame does not have to represent *each* pixel as a concrete memory allocation. Instead, they
     *must* implement `GetPixelData()` which enumerates the pixel data sequentially.
     - Each return consists of an `RGB8BPixel` and an `end_of_frame` indicator, which is set *after*
       the end of the frame is reached. In a sense, `RGB8BPixel` and `end_of_frame` are mutually
       exclusive. But for processing efficiency, they are presented as peer fields in `PixelWithStatus`.
     - The base implementation also provides a `index_` field and a `Reset()` implementation
       which allows "rewinding" a frame to re-enumerate its pixel data.
    - Three `Target` implementations are provided in the "stock" implementation:
      - `BlenderFrame`: blends two arbitrary frames into a single frame. This is used internally by
        the renderer to provide the generic transitioning effect;
      - `UniformColorFrame`: render a uniform color across the frame;
      - `ColorDotFrame`: render a color dot on a frame;
    - Generally, each custom `Target` should have a corresponding `Frame` which handles the rendering.
  - `RGB8BPixel` is a fixed pixel format for 8-bit color data in [Red, Green, Blue] order.
     - It is *agnostic* to the color ordering of the hardware, which is configurable via the `IOConfig`
       parameter of the [driver setup API](#driver-interface);
     - This allows the source code *and* the compile binary to be able to adapt to hardware expect
       different color ordering.
  - `RGBA8BPixel` is similar to `RGB8BPixel`, but has an extra 8-bit alpha channel, which denotes the
    "opacity" of the pixel's color. It is used to express semi-transparent pixels.
    - `RGBA8BBlendPixel` is a special variant of `RGBA8BPixel`. It also contains "RGB+alpla" data,
      but their values are "pre-progressed", such that redundant computation is minimized when the
      same pixels are "blended" over multiple pixels.

### Driver Implementation Notes
For those who are curious about what's under the hood.

#### Interrupt Handling
Two hardware facilities are occupied by the driver: the UART and the FRC1 timer. They both share a
single ISR, which juggles its invocations between the two:

```mermaid
graph TD;
TXIntr([UART TX FIFO Empty])
HWTIntr([HW Timer Expire])

ISREntry("ISR Entry")
ISRExit("ISR Exit")

IntrDisarm["Disable interrupt"]
FTState{{"Frame transmission state"}}
style FTState fill:#475
FrameMoreData{{"More data for the frame?"}}
RBufRecv["Receive from ring buffer"]
FUState1{{"Frame underflow state"}}
style FUState1 fill:#448
UARTSend["Convert & enqueue UART TX FIFO"]

FTStateEoF>"Frame transmission state: end-of-frame"]
style FTStateEoF fill:#475
TimeFIFODeplete[/"wakeup time = time to deplete FIFO"/]

FUState2{{"Frame underflow state"}}
style FUState2 fill:#448
FUSentinel>"Frame underflow state: sentinel"]
style FUSentinel fill:#448
TimeUnderflow[/"wakeup time = time to deplete FIFO + min reset time - ISR overhead"/]

FUActual>"Frame underflow state: underflow"]
style FUActual fill:#448
FUReset1>"Frame underflow state: normal"]
style FUReset1 fill:#448

FrameDiscard["Discard data"]
FrameDiscardComplete{{"Whole frame received?"}}
FUReset2>"Frame underflow state: normal"]
style FUReset2 fill:#448
FTIdle1>"Frame transmission state: wait-next"]
style FTIdle1 fill:#475

FrameReset[/"wakeup time = std reset time"/]
FTIdle2>"Frame transmission state: wait-next"]
style FTIdle2 fill:#475

RBufTell["Sufficient data in ring buffer?"]
FrameWait[/"wakeup time = N * std reset time (2 <= N <= 25)"/]
FTNewFrame>"Frame transmission state: frame-in-progress"]
style FTNewFrame fill:#475

TXIntr==>ISREntry
HWTIntr-->ISREntry

ISREntry==>IntrDisarm
IntrDisarm==>FTState
FTState== frame-in-progress ==>FrameMoreData
FrameMoreData== need more data ==>RBufRecv
RBufRecv== got pixel data ==>FUState1
FUState1== normal ==>UARTSend
UARTSend== arm UART interrupt ==>ISRExit

FrameMoreData-- no more data -->FTStateEoF
FTStateEoF-->TimeFIFODeplete
TimeFIFODeplete-- arm HW timer -->ISRExit

RBufRecv-. no data .->FUState2
FUState2-. normal .->FUSentinel
FUSentinel-.->TimeUnderflow
TimeUnderflow-. arm HW timer .->ISRExit

FUState2-. sentinel .->FUActual
FUState2-. underflow .->FrameReset
FUActual-.->FrameReset
FrameReset-. arm HW timer .->ISRExit

FUState1-- sentinel -->FUReset1
FUReset1-->UARTSend

FUState1-. underflow .->FrameDiscard
FrameDiscard-.->FrameDiscardComplete
FrameDiscardComplete-. Yes .->FUReset2
FrameDiscardComplete-. No .->FrameReset
FUReset2-.->FTIdle1
FTIdle1-.->RBufTell

FTState-- end-of-frame -->FTIdle2
FTIdle2-->FrameReset

FTState-. wait-next .->RBufTell
RBufTell-. No .->FrameWait
FrameWait-. arm HW timer .->ISRExit
RBufTell-. Yes .->FTNewFrame
FTNewFrame-.->RBufRecv
```

If you find the chart too confusing, there are 6 distinct operational workflows:
- **Frame transmission**: triggered by UART1 TX FIFO empty event. Will receive RGB pixel data from
  the ring buffer, convert that to level sequence, enqueue into TX FIFO; UART interrupt re-armed.
  **This is the main workflow, shown in the above chart with bold arrow connectors.**
- **Frame interruption**: triggered by UART1 TX FIFO empty event, when the frame has not been
  completely received but the ring buffer has no data; Will set HW timer a timeout of
  `time left to deplete the TX FIFO` + `min reset time` - `ISR overhead`. The idea is that, if
  there is still no data from the ring buffer when the timer expires, we will treat it as a real
  frame underflow and [drop the rest of the frame's data](#frame-underflow-handling).
  **This workflow is shown in the above chart with dotted arrow connectors.**
- **Frame completion**: triggered by UART1 TX FIFO empty event, when a frame has been completely
  received; Will set an internal flag to signal end-of-frame, an set HW timer a timeout of
  `time left to deplete the TX FIFO`.
- **Frame reset**: triggered by HW timer, when the "end-of-frame" flag is set. Will re-arm the
  timer with a timeout of `std reset time`, and set the "wait-next" flag.
- **Frame waiting**: triggered by HW timer, when the "wait-next" flag is set. Will check if
  the ring buffer has sufficient data for the next frame. If so, *directly* enters the
  **frame transmission** workflow; otherwise, re-arm the timer with an increasing multiple of
  `std reset time` (max 25 times `std reset time`).
- **Frame drop**: triggered by HW timer. Try to receive the rest of the frame's pixel data from
  the ring buffer; If the complete frame has been received, sets "wait-next" flag and *directly*
  enters **frame waiting** workflow; Otherwise, regardless of how much data received, re-arm the
  timer with a timeout of `std reset time`.
  **This workflow is shown in the above chart also with dotted arrow connectors.**

#### Ring buffer
It sits between the "user" task which [performs the rendering](#frame-rendering-api), i.e. pulling
new frames from the renderer and fetching pixel data for each frame, and [the ISR](#interrupt-handling).
It allows complex rendering operation to be performed with relaxed constraints (e.g. involving flash
reading, either for data or instructions), which cannot happen in an ISR.

The driver implementation supports two types of ring buffers: the IDF's `BYTEBUF` ring buffer, and a
block-based custom ring buffer. The default and recommended implementation is the custom ring buffer.

The IDF ring buffer implementation is more general, for example, it allows multiple concurrent senders.
However, the generality reduces its efficiency. OTOH, the custom ring buffer is tailored to the very
purpose of delivering pixels between a renderer task and the ISR, hence is more efficient.

At 160MHz with very light concurrent loads and generous jitter buffers (e.g. >1000us), there isn't a
very significant difference between the two implementations. However, at 80Mhz and pushing the buffer
size to the extreme, the custom ring buffer shows better performance and stability.

##### Pixel block size
By default the custom ring buffer uses 9 pixel blocks - each block will fit tightly into the UART
TX FIFO during frame transmission. This is the most efficient configuration.

However, because a pixel block will only be delivered to the ISR after *all* pixels are produced, if
the pixel generation computation is very expensive, producing all 9 pixels may exceed the LED's reset
time and result in frame underflow.

To mitigate that you can configure a smaller pixel block size, thereby breaking the computation into
smaller chunks, and reducing the likelihood for underflow.

However, smaller pixel block size will add some processing overhead, reducing the overall efficiency
of frame rendering and transmission.

#### FreeRTOS ticks and FPS
Currently the rendering task will sleep one tick when the renderer returned no frame in the middle
of a transition. This happens when a frame is completely rendered, but it is not yet time for the
next frame.

Since by default FreeRTOS ticks run at 100Hz, this sets up a "barrier" at 100fps. (Realistically the
rendering can reach about 105fps, because there are occasions the renderer can immediately provide
the next frame.) I believe it is sufficient for most general use.

However, if you do have short strips and wants to render at event high fps, you could configure
FreeRTOS to run ticks at higher frequency.
