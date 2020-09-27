/**
 * EMRGBLight package
 */

enum RgbColors {
    //% block=red
    Red = 1,
    //% block=orange
    Orange = 2,
    //% block=yellow
    Yellow = 3,
    //% block=green
    Green = 4,
    //% block=blue
    Blue = 5,
    //% block=indigo
    Indigo = 6,
    //% block=violet
    Violet = 7,
    //% block=purple
    Purple = 8,
    //% block=white
    White = 9
}

/**
 * Different modes for RGB or RGB+W RGBLight LHRGBLight
 */
enum EMRGBPixelMode {
    //% block="RGB (GRB format)"
    RGB = 0,
    //% block="RGB+W"
    RGBW = 1,
    //% block="RGB (RGB format)"
    RGB_RGB = 2
}

/**
 * RGBLight Functions
 */
namespace EMRGBLight {
    //% shim=sendBufferAsm
    //% parts="RGBLight"
    function sendBuffer(buf: Buffer, pin: DigitalPin) {

    }

    /**
    * A RGBLight class
    */
    export class EmakefunRGBLight {
        buf: Buffer;
        pin: DigitalPin;
        // TODO: encode as bytes instead of 32bit
        brightness: number;
        start: number; // start offset in LED strip
        _length: number; // number of LEDs
        _mode: EMRGBPixelMode;

        setBrightness(brightness: number): void {
            this.brightness = brightness & 0xff;
            this.easeBrightness();
        }

        setPin(pin: DigitalPin): void {
            this.pin = pin;
            pins.digitalWritePin(this.pin, 0);
            // don't yield to avoid races on initialization
        }

        setPixelColor(pixeloffset: number, rgb: RgbColors): void {
            if (pixeloffset == this._length)//全部
            {
                for (let i = 0; i < this._length; i++)
                {
                    this.setPixelRGB(i, rgb);     
                }
            }
            else
            {
                this.setPixelRGB(pixeloffset, rgb);
            }
        }

        private setPixelRGB(pixeloffset: number, rgb: RgbColors): void {
            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;
            let tureRgb = 0;
            switch (rgb)
            {
                case RgbColors.Red:
                    tureRgb = 0xFF0000;
                    break;    

                case RgbColors.Orange:
                    tureRgb = 0xFFA500;    
                    break;    

                case RgbColors.Yellow:
                    tureRgb = 0xFFFF00;
                    break;    
                    
                case RgbColors.Green:
                    tureRgb = 0x00FF00;    
                    break;    

                    case RgbColors.Blue:
                    tureRgb = 0x0000FF;
                    break;    
                    
                case RgbColors.Indigo:
                    tureRgb = 0x4b0082;    
                    break;    

                case RgbColors.Violet:
                    tureRgb = 0x8a2be2;
                    break;    
                    
                case RgbColors.Purple:
                    tureRgb = 0xFF00FF;    
                    break;   

                case RgbColors.White:
                    tureRgb = 0xFFFFFF;    
                    break;   
            }

            let stride = this._mode === EMRGBPixelMode.RGBW ? 4 : 3;
            pixeloffset = (pixeloffset + this.start) * stride;

            let red = unpackR(tureRgb);
            let green = unpackG(tureRgb);
            let blue = unpackB(tureRgb);

            let br = this.brightness;
            if (br < 255) {
                red = (red * br) >> 8;
                green = (green * br) >> 8;
                blue = (blue * br) >> 8;
            }
            this.setBufferRGB(pixeloffset, red, green, blue)
        }

        private setBufferRGB(offset: number, red: number, green: number, blue: number): void {
            if (this._mode === EMRGBPixelMode.RGB_RGB) {
                this.buf[offset + 0] = red;
                this.buf[offset + 1] = green;
            } else {
                this.buf[offset + 0] = green;
                this.buf[offset + 1] = red;
            }
            this.buf[offset + 2] = blue;
        }

        show() {
            sendBuffer(this.buf, this.pin);
        }

        clear(): void {
            const stride = this._mode === EMRGBPixelMode.RGBW ? 4 : 3;
            this.buf.fill(0, this.start * stride, this._length * stride);
            this.show();
        }

        easeBrightness(): void {
            const stride = this._mode === EMRGBPixelMode.RGBW ? 4 : 3;
            const br = this.brightness;
            const buf = this.buf;
            const end = this.start + this._length;
            const mid = this._length / 2;
            for (let i = this.start; i < end; ++i) {
                const k = i - this.start;
                const ledoffset = i * stride;
                const br = k > mid ? 255 * (this._length - 1 - k) * (this._length - 1 - k) / (mid * mid) : 255 * k * k / (mid * mid);
                const r = (buf[ledoffset + 0] * br) >> 8; buf[ledoffset + 0] = r;
                const g = (buf[ledoffset + 1] * br) >> 8; buf[ledoffset + 1] = g;
                const b = (buf[ledoffset + 2] * br) >> 8; buf[ledoffset + 2] = b;
                if (stride == 4) {
                    const w = (buf[ledoffset + 3] * br) >> 8; buf[ledoffset + 3] = w;
                }
            }
        }
    }
    
    export function create(pin: DigitalPin, numleds: number, mode: EMRGBPixelMode): EmakefunRGBLight {
        let light = new LHRGBLight();
        let stride = mode === EMRGBPixelMode.RGBW ? 4 : 3;
        light.buf = pins.createBuffer(numleds * stride);
        light.start = 0;
        light._length = numleds;
        light._mode = mode;
        light.setBrightness(255);
        light.setPin(pin);
        return light;
    }

    function packRGB(a: number, b: number, c: number): number {
        return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
    }
    function unpackR(rgb: number): number {
        let r = (rgb >> 16) & 0xFF;
        return r;
    }
    function unpackG(rgb: number): number {
        let g = (rgb >> 8) & 0xFF;
        return g;
    }
    function unpackB(rgb: number): number {
        let b = (rgb) & 0xFF;
        return b;
    }
}
