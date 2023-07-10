#import <AVFoundation/AVFoundation.h>
#import <Foundation/Foundation.h>
void requestCameraPermission() {
    if ([AVCaptureDevice respondsToSelector:@selector(requestAccessForMediaType: completionHandler:)]) {
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
            if (!granted) {
                NSLog(@"Access to camera denied.");
            }
        }];
    }
}
