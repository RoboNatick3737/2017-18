public class NativeTest
{
    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    // Should not be red if CMake is configured correctly.
    public native String printHello();
}
