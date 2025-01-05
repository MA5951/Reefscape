
package com.ma5951.utils.Vision.Sim;


public class CameraTypes {
    public enum CameraType {
        LLG3(1280, 800, 86 ),
        LL3(1280, 960, 70),
        LL2_PLUS(640, 480, 50);
    
        private final int width;
        private final int height;
        private final int fov;
    
        CameraType(int width, int height, int fov) {
            this.width = width;
            this.height = height;
            this.fov = fov;
        }
    
        public int getWidth() {
            return width;
        }
    
        public int getHeight() {
            return height;
        }
    
        public int getFov() {
            return fov;
        }
    }
    
}
