import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import java.lang.*;

/**
 * Pi class utilizes Monte Carlo method to determine
 * the value of Pi by simulating throwing darts
 * at a dartboard. 
 */
public class Pi {
    private static AtomicLong count;
    private final static double MIN_BOUND = 0.0;
    private final static double MAX_BOUND = 1.0;
    private final static int DEFAULT_THREADS = 1;
    private final static int DEFAULT_ITERATIONS = 1000;

    
    static class PiThread implements Runnable {
        private final long iterations;
        private final CountDownLatch signal;

        /**
         * Pi Thread Constructor
         * @param iterations  Number of times to throw dart
         * @param signal      CountDownLatch allows main thread to pause until all worker threads finish
         */
        PiThread(long iterations, CountDownLatch signal) {
            this.iterations = iterations;
            this.signal     = signal;
        }

        /**
         * This thread *throws* a dart at a dartboard by randomly 
         * selecting values between 0.000 & 1.000. If the dart lies
         * within desired quadrant, an atomic counter is incremented.
         * This also decrements CountDownLatch to indicate that
         * this thread has finished executing.
         */
        public void run() {
            //System.out.println("Running Thread: " + Thread.currentThread().getId());
            double x, y;
            for (int j=0; j<iterations; j++){
                x = ThreadLocalRandom.current().nextDouble(MIN_BOUND, MAX_BOUND);
                y = ThreadLocalRandom.current().nextDouble(MIN_BOUND, MAX_BOUND);
                if ( (x*x + y*y) < 1 ) {
                    count.getAndIncrement();
                }
            }
            signal.countDown();
        }
    }

    public static void main(String args[]) {
        long numThreads = DEFAULT_THREADS;
        long iterations = DEFAULT_ITERATIONS;
        try {
            numThreads = Long.parseLong(args[0]);    
            iterations = Long.parseLong(args[1]);
        } catch (NumberFormatException nfe) {
            System.out.println("Usage:\n\tjava Pi <num_threads> <iterations>");
            System.exit(-1);
        }
        count = new AtomicLong();
        CountDownLatch signal = new CountDownLatch( (int)numThreads);

        //Initalize N Threads to do Iterations/N work
        for (int i=0; i<numThreads; i++){
            new Thread(new PiThread(iterations/numThreads, signal)).start();
        }

        try {
            //System.out.println("Main Thread: " + Thread.currentThread().getId());
            //System.out.println("Waiting for worker threads...");
            signal.await();
        } catch (InterruptedException ie){
            System.out.println("Thread interrupted");
            System.exit(-1);
        }
        
        //System.out.print("Calculating results...\n");
        System.out.println("Total = " + iterations);
        System.out.println("Inside = " + count.get());
        System.out.println("Ratio = " + (double)count.get()/iterations);
        System.out.println("Pi = " + ((double)count.get()/iterations)*4);

        
    }

}