import time

while True:
    t1 = time.perf_counter()

    time.sleep(1/9)

    t2 = time.perf_counter()

    elapsedTime = t2-t1      
    fps = 1/elapsedTime     
    print(t1, t2)
    print(fps)