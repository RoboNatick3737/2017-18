#include <jni.h>
#include <string>

extern "C"
JNIEXPORT jstring

JNICALL
// Currently not configured but will be
Java_package_Class_filterForCryptobox(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
