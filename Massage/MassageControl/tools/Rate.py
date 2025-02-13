import time
import asyncio

class Rate:
    def __init__(self, hz):
        self.interval = 1.0 / hz
        self.last_time = time.monotonic()
        self.start_time = self.last_time
    
    def sleep(self,print_duration=False):
        now = time.monotonic()
        sleep_duration = self.interval - (now - self.last_time)
        if sleep_duration > 0:
            if print_duration:
                print(f"睡眠时间: {sleep_duration:.4f} 秒")
            time.sleep(sleep_duration)
        self.last_time = time.monotonic()

    async def async_sleep(self):
        now = time.monotonic()
        sleep_duration = self.interval - (now - self.last_time)
        if sleep_duration > 0:
            # print(f"睡眠时间: {sleep_duration:.4f} 秒")
            await asyncio.sleep(sleep_duration)
        self.last_time = time.monotonic()

    def remaining(self):
        now = time.monotonic()
        remaining_time = self.interval - (now - self.last_time)
        return max(0.0, remaining_time)

    def cycle_time(self):
        now = time.monotonic()
        cycle_duration = now - self.start_time
        self.start_time = now
        return cycle_duration
    
    def to_sec(self):
        return self.interval

if __name__ == "__main__":
    # 示例用法：同步代码
    def main_sync():
        rate = Rate(2)  # 2 赫兹的循环频率（每次迭代0.5秒）
        for i in range(10):
            print(f"同步迭代 {i}")
            print(f"剩余时间: {rate.remaining():.4f} 秒")
            print(f"上一个循环周期时间: {rate.cycle_time():.4f} 秒")
            rate.sleep()

    # 示例用法：异步代码
    async def main_async():
        rate = Rate(2)  # 2 赫兹的循环频率（每次迭代0.5秒）
        for i in range(10):
            print(f"异步迭代 {i}")
            print(f"剩余时间: {rate.remaining():.4f} 秒")
            print(f"上一个循环周期时间: {rate.cycle_time():.4f} 秒")
            await rate.async_sleep()

    main_sync()

    asyncio.run(main_async())

