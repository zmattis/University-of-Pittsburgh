/*************************************************************************
 *  Compilation:  javac variableLZW.java
 *  Execution:    java variableLZW - n < input.txt   (compress, standard execution)
 *  Execution:    java variableLZW - r < input.txt   (compress, reset codebook)
 *  Execution:    java variableLZW - m < input.txt   (compress, monitor mode)

 *  Execution:    java variableLZW + < input.txt   (expand)
 *  Dependencies: BinaryIn.java BinaryOut.java
 *
 *  Compress or expand binary input from standard input using LZW compression.
 *
 *  Standard -- Fills up codebook to maximum and continues to use throughout the rest of the file
 *  Reset -- Resets the codebook once it has been filled to capacity
 *  Monitor -- Resets the codebook once it is full, only if the ratio of compression exceeds 1.1
 *
 *  WARNING: STARTING WITH ORACLE JAVA 6, UPDATE 7 the SUBSTRING
 *  METHOD TAKES TIME AND SPACE LINEAR IN THE SIZE OF THE EXTRACTED
 *  SUBSTRING (INSTEAD OF CONSTANT SPACE AND TIME AS IN EARLIER
 *  IMPLEMENTATIONS).
 *
 *  See <a href = "http://java-performance.info/changes-to-string-java-1-7-0_06/">this article</a>
 *  for more details.
 *
 *************************************************************************/

public class variableLZW {
    private static final int R = 256;        // number of input chars
    private static final int MIN_CW_NUM = 512;  // Min (2^W)
    private static final int MAX_CW_NUM = 65536;  // Max (2^W)
    private static int L = MIN_CW_NUM;       // number of codewords = 2^W
    private static final int MIN_WIDTH = 9;         // Min num of bits
    private static final int MAX_WIDTH = 16;      // Max num of bits
    private static int W = MIN_WIDTH;         // codeword width

    private static String mode = "n";       // LZW mode
    private static char compressionType;    //LZW mode flag
    private static int sizeU = 0;         //Uncompressed size
    private static int sizeC = 0;           //Compressed size
    private static double compressionRatio = 0.0;   //Compression ratio
    private static double oldRatio = 0.0;         //Old Compression ratio
    private static boolean hasRatio = false;      // Ratio flag


    /*  LZW compression algorithm */
    public static void compress() {
        BinaryStdOut.write(mode, 8);            // Output compressionType to SOF
        String input = BinaryStdIn.readString();    // Read Input file from i/o redirection
        TST<Integer> st = new TST<Integer>();       // Create ternary search trie
        for (int i = 0; i < R; i++)                 // Initialize TST
            st.put("" + (char) i, i);
        int code = R+1;  // R is codeword for EOF

        while (input.length() > 0) {
            L = (int)Math.pow(2,W);       // 2^W
            String s = st.longestPrefixOf(input);  // Find max prefix match s.

            sizeU += s.length() * 8;

            BinaryStdOut.write(st.get(s), W);      // Print s's encoding.

            sizeC += W;
            compressionRatio = sizeU/sizeC;     //Update Compression Ratio

            int t = s.length();
            if (t < input.length() && code < L){    // Add s + 1 char to symbol table.
              st.put(input.substring(0, t + 1), code++);
            }


            /* Increase CodeWord Width from 9 bits to 16 bits */
            if ( (W<MAX_WIDTH) && (code == (int)Math.pow(2,W)) ){
              W++;          // Increment codeword width
              L = (int)Math.pow(2,W);   //Calculate new number of codewords
              st.put(input.substring(0, t+1), code++);    //Add tp
            }

            /* Reset Mode -- When all codewords have been used (2^16), reset codebook */
            if ( (mode.equals("r")) && (code == MAX_CW_NUM) ){
              st = new TST<Integer>();      //reset dictionary
              for (int i = 0; i < R; i++)                 // Initialize TST
                  st.put("" + (char) i, i);
              code = R+1;       // EOF
              W = MIN_WIDTH;        // 9 bits
              L = MIN_CW_NUM;       // 2^9
            }

            /* Monitor Mode -- monitor compression ratio when full and reset when old/new exceeds 1.1 */
            if ( mode.equals("m") && (code == MAX_CW_NUM) ){
                if(!hasRatio){                    // No compression ratio state
                  oldRatio = compressionRatio;    // Set to compressionRatio
                  hasRatio = true;                // Ratio flag - true
                }

                if ( (oldRatio/compressionRatio) > 1.1 ){     //Compression ratio exceeds 1.1
                  st = new TST<Integer>();                    // Reset codebook
          				for (int i = 0; i < R; i++) {
          					st.put("" + (char)i, i);
          				}
          				code = R + 1;
          				W = MIN_WIDTH;
          				L = MIN_CW_NUM;
          				oldRatio = 0;
          				compressionRatio = 0;
          				hasRatio = false;
                }
            }

            input = input.substring(t);            // Scan past s in input.
          }

        BinaryStdOut.write(R, W);
        BinaryStdOut.close();
    }

    /* LZW expansion algorithm */
    public static void expand() {
        compressionType = BinaryStdIn.readChar(8);      //Read first char of file to determine compression mode
        System.err.println(compressionType);
        String[] st = new String[MAX_CW_NUM];
        int i; // next available codeword value

        // initialize symbol table with all 1-character strings
        for (i = 0; i < R; i++)
            st[i] = "" + (char) i;
        st[i++] = "";                        // (unused) lookahead for EOF

        int codeword = BinaryStdIn.readInt(W);
        if (codeword == R) return;           // expanded message is empty string
        String val = st[codeword];

        while (true) {
            BinaryStdOut.write(val);
            sizeU = val.length() * 8;
            codeword = BinaryStdIn.readInt(W);
            sizeC += W;
            compressionRatio = sizeU/sizeC;
            //System.err.println(codeword); // Debug
            if (codeword == R) break;
            String s = st[codeword];
            if (i == codeword) s = val + val.charAt(0);   // special case hack

            //System.err.println(L);
            if (i < (L-1) ) st[i++] = val + s.charAt(0);

            /* Increase CodeWord Width from 9 bits to 16 bits */
            if ( (W < MAX_WIDTH) && (i == (L - 1)) ) {
        			st[i++] = val + s.charAt(0);
        			W++;     //Increment codeword width
        			L = (int)Math.pow(2, W);     // 2^W
        		}

            /* Reset Mode -- reset codebook once dictionary is completely read */
            if ( (compressionType == 'r')  && (i == MAX_CW_NUM) ) {
                W = MIN_WIDTH;
                L = MIN_CW_NUM;
                st = new String[MAX_CW_NUM];
                for (i = 0; i < R; i++) {
                  st[i] = "" + (char)i;
                }
                st[i++] = "";						// (unused) lookahead for EOF
                codeword = BinaryStdIn.readInt(W);
                if (codeword == R) break;			// expanded message is empty string
                val = st[codeword];
            }


            /* Monitor Mode -- Reset codeboook once ratio exceeds 1.1 */
            if (compressionType == 'm' && i == MAX_CW_NUM) {
        			if (!hasRatio) {
        				oldRatio = compressionRatio;
        				hasRatio = true;
        			}
        			if ( (oldRatio/compressionRatio) > 1.1 ) {
        				W = MIN_WIDTH;
        				L = MIN_CW_NUM;
        				st = new String[MAX_CW_NUM];
        				for (i = 0; i < R; i++) {
        					st[i] = "" + (char)i;
        				}
        				st[i++] = "";
        				codeword = BinaryStdIn.readInt(W);
        				if (codeword == R) break;
        				val = st[codeword];

        				oldRatio = 0;
        				compressionRatio = 0;
        				hasRatio = false;
        			}
            }
            val = s;
        }
        BinaryStdOut.close();
    }


    /* Command Line Parsing for Compress/Expand */
    public static void main(String[] args) {
        if      (args[0].equals("-")) {
            if      (args[1].equals("n")){    // Standard (do nothing) mode
              mode = args[1];
              compress();
            }
            else if (args[1].equals("r")){    // Reset mode
              mode = args[1];
              compress();
            }
            else if (args[1].equals("m")){    // Monitor mode
              mode = args[1];
              compress();
            }
            else throw new IllegalArgumentException("Illegal command line argument");
        }
        else if (args[0].equals("+")) expand();
        else throw new IllegalArgumentException("Illegal command line argument");
    }

}
