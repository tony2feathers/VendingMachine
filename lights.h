#ifndef LIGHTS
#define LIGHTS
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <algorithm> // Add this line to include the <algorithm> header
#include <algorithm> // Add this line to include the <algorithm> header

// Paterns to be used for light functions
enum pattern
{
    none,
    runningLights,
    theaterChase,
    colorWipe,
    colorWave,
    cylonEye,
    scanner,
    fade,
    acceleratingSequence,
    flash
};

// Pattern directions
enum direction
{
    forward,
    reverse
};

// Class for LED patterns
class NeoPatterns : public Adafruit_NeoPixel
{
private:
    bool patternFinished = false;
    bool isReadyToUpdate()
    {
        return (millis() - lastUpdate) > Interval;
    }
    float calculateBrightness(int distance, int maxDistance)
    {
        return 1.0 - (float)distance / maxDistance;
    }

    void updateSegmentPixels(int start, int len, uint32_t color, std::function<float(int, int)> brightnessCalc)
    {
        for (int i = start; i < start + len; i++)
        {
            int distance = abs(i - (start + len / 2));
            float brightness = brightnessCalc(distance, len / 2);
            setPixelColor(i, DimColor(Color(
                                 brightness * Red(color),
                                 brightness * Green(color),
                                 brightness * Blue(color))));
        }
        show();
    }

public:
    // Member Variables:
    pattern ActivePattern; // which pattern is running
    direction Direction;   // durection to run the pattern

    int Interval;             // milliseconds between updates
    unsigned long lastUpdate; // last update of position

    uint32_t Color1, Color2; // what colors are in use
    uint16_t TotalSteps;     // total number of steps in the pattern
    uint16_t Index;          // current step within the pattern
    uint16_t Reps;           // number of repetitions to animate
    uint16_t EyeSize;        // size of the eye for cylon effects
    uint16_t TailLengthA;    // length of the tail for scanner effects
    uint16_t TailLengthB;    // length of the tail for scanner effects

    int segmentStart;  // Member variable to store the starting pixel
    int segmentLen;    // Member variable to store the length of a given segment
    int segmentStartB; // Member variable to store the starting pixel
    int segmentLenB;   // Member variable to store the length of a given segment

    void (*OnComplete)(NeoPatterns *); // Callback on completion of pattern

    // Constructor - calls base-class constructor to initialize the strip
    NeoPatterns(uint16_t pixels, uint16_t pin, uint16_t type, void (*callback)(NeoPatterns *))
        : Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    void markPatternFinished()
    {
        patternFinished = true;
    }

    bool isPatternFinished() const
    {
        return patternFinished;
    }

    // Update the pattern
    void Update()
    {
        if (isReadyToUpdate())
        {
            lastUpdate = millis();

            switch (ActivePattern)
            {
            case runningLights:
                RunningLightsUpdate();
                break;
            case theaterChase:
                TheaterChaseUpdate();
                break;
            case colorWipe:
                ColorWipeUpdate();
                break;
            case colorWave:
                ColorWaveUpdate();
                break;
            case cylonEye:
                CylonEyeUpdate();
                break;
            case scanner:
                ScannerUpdate();
                break;
            case fade:
                FadeUpdate();
                break;
            case acceleratingSequence:
                AcceleratingSequenceUpdate();
                break;
            case flash:
                FlashUpdate();
                break;
            default:
                break;
            }
        }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == forward)
        {
            Index++;
            if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(this); // call the completion callback
                }
            }
        }
        else // Direction == reverse
        {
            if (Index == 0)
            {
                Index = TotalSteps - 1; // Reset to the last pixel in reverse
                if (OnComplete != NULL)
                {
                    OnComplete(this); // Call the completion callback
                }
            }
            else
            {
                --Index; // Decrement after checking
            }
        }
    }

    // Reverse the pattern direction
    void Reverse()
    {
        if (Direction == forward)
        {
            Direction = reverse;
            Index = TotalSteps - 1;
        }
        else
        {
            Direction = reverse;
            Index = 0;
        }
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = forward)
    {
        ActivePattern = theaterChase;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
        patternFinished = false;
    }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }

    // Initialize for a Flash pattern
    void Flash(uint32_t color, int interval, int start, int len, direction dir = forward)
    {
        ActivePattern = flash;
        Interval = interval;
        TotalSteps = len; // Ensure this is correct for your expectations
        Color1 = color;
        Index = 0; // Start at 0
        Direction = dir;
        segmentStart = start;
        segmentLen = len;
        patternFinished = false;
    }

    void FlashUpdate()
    {
        // Toggle every other pixel based on Index and Interval
        for (int i = segmentStart; i < segmentStart + segmentLen; i++)
        {
            if ((i + Index) % 2 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, 0); // Turn off the pixel
            }
        }
        show();      // Update the strip
        Increment(); // Increment the index for toggling
    }

    // Initialize for Running LightsLen, uint32_t color, int waveDelay, dir
    void RunningLights(uint32_t color, uint8_t WaveDelay, int start, int len, direction dir = forward)
    {
        ActivePattern = runningLights;
        Color1 = color;
        Interval = WaveDelay;
        Direction = dir;
        TotalSteps = len * 2;
        Index = 0;
        segmentStart = start;
        segmentLen = len;
        patternFinished = false;
    }

    // Update the RunningLights pattern animation frame
    void RunningLightsUpdate()
    {
        float brightness = ((sin(Index) * 127 + 128) / 255.0);

        for (int j = segmentStart; j < segmentStart + segmentLen; j++)
        {
            setPixelColor(j, Adafruit_NeoPixel::Color(
                                 brightness * ((Color1 >> 16) * 0xFF),
                                 brightness * ((Color1 >> 8) * 0xFF),
                                 brightness * (Color1 & 0xFF)));
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, int start, int len, direction dir = forward)
    {
        ActivePattern = colorWipe;
        Color1 = color;
        Interval = interval;
        TotalSteps = len;
        segmentStart = start;
        segmentLen = len;
        Index = 0;
        Direction = dir;
        patternFinished = false;
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        if (Direction == forward) // Wipe forward
        {
            for (int k = segmentStart; k < segmentStart + segmentLen; k++)
            {
                setPixelColor(k, Color1);
            }
            show();      // Turn on the current LED
            Increment(); // Increment the animation
        }
        else // Wipe backwards
        {
            for (int l = segmentStart + segmentLen; l > segmentStart; l--)
            {
                setPixelColor(l, Color1);
            }
            show();      // Turn on the current LED
            Increment(); // Increment the animation
        }
    }

    // Initialize for a wave effect (lights the pixels sequentially then turns them off, then reverses direction)
    void ColorWave(uint32_t color, int start, int len, int speed, int reps, direction dir = forward)
    {
        ActivePattern = colorWave;
        Interval = speed;
        segmentLen = len;
        segmentStart = start;
        Reps = reps;
        TotalSteps = segmentLen * Reps;
        Color1 = color;
        Index = 0;
        Direction = dir;
        patternFinished = false;
    }

    // Update the Color Wave Pattern
    void ColorWaveUpdate()
    {
        if (Direction == forward) // Wave forward
        {
            for (int i = segmentStart; i < segmentStart + segmentLen; i++)
            {
                setPixelColor(i, Color1);
                show();
            }
            for (int j = segmentStart; j < segmentStart + segmentLen; j++)
            {
                setPixelColor(j, (0, 0, 0));
                show();
                Increment();
            }
            if (Index == TotalSteps)
            {
                Reverse();
            }
        }
        else if (Direction == reverse) // Wave in reverse
        {
            // Reverse phase: Turn pixels on in reverse order
            for (int k = segmentStart + segmentLen - 1; k >= segmentStart; k--)
            {
                setPixelColor(k, Color1);
                show();
            }

            // Reverse phase: Now turn them off in reverse order
            for (int l = segmentStart + segmentLen - 1; l >= segmentStart; l--)
            {
                setPixelColor(l, 0);
                show();
                Increment();
            }
            if (Index == TotalSteps)
            {
                Reverse();
            }
        }
    }

    // Initialize a cylon effect (with an eye)
    void CylonEye(uint32_t color, uint8_t interval, int start, int len, int eye, int reps, direction dir = forward)
    {
        ActivePattern = cylonEye;
        Interval = interval;
        Color1 = color;
        Reps = reps;
        segmentLen = len;
        segmentStart = start;
        TotalSteps = segmentLen * Reps;
        Index = 0;
        Direction = dir;
        EyeSize = min(eye, segmentLen / 2);
        patternFinished = false;
    }

    // Update function for cylon effect
    void CylonEyeUpdate()
    {
        float brightnessCache[segmentLen];
        for (int i = 0; i < segmentLen; i++)
        {
            brightnessCache[i] = calculateBrightness(abs(i - (segmentLen / 2)), EyeSize);
        }

        for (int i = segmentStart; i < segmentStart + segmentLen; i++)
        {
            setPixelColor(i, DimColor(Color(
                                 brightnessCache[i - segmentStart] * Red(Color1),
                                 brightnessCache[i - segmentStart] * Green(Color1),
                                 brightnessCache[i - segmentStart] * Blue(Color1))));
        }
        show();
        Increment();
    }

    // Initialize for a scanner
    void Scanner(uint32_t color1, uint32_t color2, uint8_t interval, int startA, int lenA, int tailLengthA, int startB, int lenB, int tailLengthB, direction dir = forward)
    {
        ActivePattern = scanner;
        Interval = interval;
        Color1 = color1;
        Color2 = color2;
        TotalSteps = (lenA + tailLengthA) * 2; // Adjust the multiplier to control the length of the tail
        segmentLen = lenA;
        segmentStart = startA;
        segmentLenB = lenB;
        segmentStartB = startB;
        Index = 0;
        Direction = forward;
        TailLengthA = tailLengthA;
        TailLengthB = tailLengthB;
        patternFinished = false;
    }

    // Update function for the scanner with a fading tail
    // ScannerUpdate: Animates two segments with fading tails moving in opposite directions.
    // - TailBrightness: Calculates the brightness of tail pixels based on distance from the active pixel.
    // - Segment A and Segment B are updated independently using the `updateSegmentPixels` function.

    void ScannerUpdate()
    {
        auto tailBrightness = [this](int distance, int tailLength) -> float
        {
            if (distance > tailLength) return 0.0; // No brightness outside the tail range
            return (float)(tailLength - distance) / tailLength;
        };

        updateSegmentPixels(segmentStart, segmentLen, Color1, tailBrightness);
        updateSegmentPixels(segmentStartB, segmentLenB, Color2, tailBrightness);
        Increment();
    }

    // Initialize for a fade effect
    void Fade(uint32_t color1, uint32_t color2, uint8_t interval, int start, int len, direction dir = forward)
    {
        ActivePattern = fade;
        Interval = interval;
        segmentLen = len;
        segmentStart = start;
        TotalSteps = segmentLen;
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
        patternFinished = false;
    }

    // Update the fade effect
    void FadeUpdate()
    {
        if (TotalSteps == 0)
            return; // Prevent division-by-zero

        // Calculate linear inerpolation between the colors
        // Optomise the order of operations to min truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;

        ColorSet(Color(red, green, blue), segmentStart, segmentLen);
        show();
        Increment();
    }

    void AcceleratingSequence(uint32_t color, int start, int len, direction dir = forward)
    {
        ActivePattern = acceleratingSequence;
        Color1 = color;
        Interval = 200;   // Start with a slow speed, 1000 milliseconds interval
        TotalSteps = len; // Total steps equals the length of the LED strip part
        Index = 0;
        segmentStart = start;
        segmentLen = len;
        Direction = dir;
        patternFinished = false;
    }

    void AcceleratingSequenceUpdate()
    {
        // Clear previous state
        fill(0);

        // Set the pixel color based on direction
        int pixelIndex = (Direction == forward) ? (segmentStart + Index) : (segmentStart + segmentLen - 1 - Index);
        setPixelColor(pixelIndex, Color1);

        // Show updates
        show();

        // Increment or decrement for next round based on direction
        Increment();

        // Accelerate: decrease interval to a minimum limit
        Interval = max(20, Interval - 5); // Decrease interval, minimum 20ms
    }

    // Calculate 50% dimmed version of a color used by scannerUpdate
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color, int start, int len)
    {
        for (int i = start; i < start + len; i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }

    // Input a value 0 to 255 to get a color value
    // The colors are a transition r - g - b - back to r
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85)
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        else if (WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        WheelPos -= 170;
        return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }

    // Default callback function that does nothing
    void ambientCallback(NeoPatterns *instance)
    {
        // Do nothing
        Serial.println("This is a message for the ambient callback function");
        instance->markPatternFinished();
    }
};
#endif // LIGHTS