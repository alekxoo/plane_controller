#include "decoder.h"
#include "fifo_format.h"
#include "../inc/tm4c123gh6pm.h"

#define SYSTEM_CLOCK 80000000
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 1024
#define FREQ_THRESHOLD 50
#define FREQ_THRESHOLD_HIGH 500

// Define frequency ranges for each symbol
#define FREQ_RANGE_SYMBOL0 0
#define FREQ_RANGE_SYMBOL1 330
#define FREQ_RANGE_SYMBOL2 1000
#define FREQ_RANGE_SYMBOL3 660

#define SYNC_FREQ 5000
#define SYNC_DURATION 0.01 // seconds
#define SYNC_TOLERANCE 0.1 // 10% tolerance for sync interruptions

Fifo sampleFifo;
uint16_t samples[BUFFER_SIZE];
volatile uint32_t sampleIndex = 0;
volatile uint32_t detected_freq;

volatile uint8_t is_synced = 0;
volatile uint32_t sync_start_time = 0;
volatile uint32_t message_start_time = 0;
volatile uint32_t sync_duration = 0;
volatile uint32_t last_sync_time = 0;

void Timer2A_Init(void);
void ADC0_Init(void);
uint32_t simple_frequency_detection(uint16_t* buffer, uint32_t buffer_size);

void Decoder_Init(void) {
    Fifo_Init(&sampleFifo);
    Timer2A_Init();
    ADC0_Init();

    // Initialize GPIO for LED outputs
    SYSCTL_RCGCGPIO_R |= 0x30;  // Enable clock for Port E and F
    while((SYSCTL_PRGPIO_R & 0x30) == 0){};  // Wait for clock to stabilize
    GPIO_PORTE_DIR_R |= 0x08;  // Set PE3 as output
    GPIO_PORTE_DEN_R |= 0x08;  // Enable digital I/O on PE3
    GPIO_PORTF_DIR_R |= 0x0E;  // Set PF1, PF2, PF3 as outputs
    GPIO_PORTF_DEN_R |= 0x0E;  // Enable digital I/O on PF1, PF2, PF3
}

uint32_t Decoder_Process(void) {
    uint16_t sample;
    uint32_t detected_freq;
    uint32_t current_time = TIMER2_TAV_R;
    static uint32_t last_sync_check_time = 0;

    // Fill the sample buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        while (!Fifo_Get(&sampleFifo, (uint16_t*)&sample)) {
            // Wait for samples
        }
        samples[i] = sample;
    }

    // Perform simple frequency detection
    detected_freq = simple_frequency_detection(samples, BUFFER_SIZE);

    // Check for sync tone if not synced
    if (!is_synced) {
        if (abs(detected_freq - SYNC_FREQ) < FREQ_THRESHOLD_HIGH) {
            if (sync_start_time == 0) {
                sync_start_time = current_time;
                last_sync_time = current_time;
            } else {
                sync_duration += current_time - last_sync_time;
                last_sync_time = current_time;

                if (sync_duration >= (SYNC_DURATION * SAMPLE_RATE)) {
                    is_synced = 1;
                    message_start_time = current_time;
                    sync_start_time = 0;
                    sync_duration = 0;
                    return 0; // Synced, but no symbol yet
                }
            }
        } else {
            // Check if we're within tolerance
            if (current_time - last_sync_time > (SYNC_TOLERANCE * SYNC_DURATION * SAMPLE_RATE)) {
                // Outside tolerance, reset sync
                sync_start_time = 0;
                sync_duration = 0;
            }
        }
        return 0; // Not synced, don't process symbols
    }

    // Check if we need to reset is_synced (every 1 second)
    if (current_time - last_sync_check_time >= 0.01*SAMPLE_RATE) {
        last_sync_check_time = current_time;
        if (abs(detected_freq - SYNC_FREQ) >= 0) {
            is_synced = 0;
            return 0; // Lost sync, don't process symbols
        }
    }

    // Determine which symbol was sent based on the detected frequency
    if (detected_freq < 100) {
        // No button pressed
        GPIO_PORTE_DATA_R &= ~0x08; // Turn off PE3
        GPIO_PORTF_DATA_R &= ~0x0E; // Turn off PF1, PF2, PF3
        return 0;
    } else if (abs(detected_freq - FREQ_RANGE_SYMBOL1) < FREQ_THRESHOLD) {
        // SW1 pressed
        GPIO_PORTE_DATA_R |= 0x08; // Turn on PE3
        GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x02; // Turn on PF1, turn off PF2, PF3
        return 1;
    } else if (abs(detected_freq - FREQ_RANGE_SYMBOL2) < FREQ_THRESHOLD) {
        // SW2 pressed
        GPIO_PORTE_DATA_R |= 0x08; // Turn on PE3
        GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x04; // Turn on PF2, turn off PF1, PF3
        return 2;
    } else if (abs(detected_freq - FREQ_RANGE_SYMBOL3) < FREQ_THRESHOLD) {
        // Both buttons pressed
        GPIO_PORTE_DATA_R |= 0x08; // Turn on PE3
        GPIO_PORTF_DATA_R |= 0x0E; // Turn on PF1, PF2, PF3
        return 3;
    }

    return 0;
}

void Timer2A_Init(void) {
    SYSCTL_RCGCTIMER_R |= 0x04;        // Enable clock for Timer2
    TIMER2_CTL_R = 0x00000000;         // Disable Timer2 during setup
    TIMER2_CFG_R = 0x00000000;         // Configure Timer2 as 32-bit timer
    TIMER2_TAMR_R = 0x00000002;        // Configure for periodic mode
    TIMER2_TAILR_R = SYSTEM_CLOCK / SAMPLE_RATE - 1; // Set interval for 44.1kHz sampling
    TIMER2_IMR_R |= 0x00000001;        // Enable timeout interrupt
    TIMER2_CTL_R |= 0x00000001;        // Enable Timer2
    NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000; // Priority 4
    NVIC_EN0_R = 1<<23;                // Enable interrupt 23 in NVIC
}

void ADC0_Init(void) {
    SYSCTL_RCGCADC_R |= 0x01;     // Activate ADC0
    SYSCTL_RCGCGPIO_R |= 0x10;    // Activate clock for Port E
    while((SYSCTL_PRGPIO_R&0x10) == 0){};
    GPIO_PORTE_DIR_R &= ~0x04;    // Make PE2 input
    GPIO_PORTE_AFSEL_R |= 0x04;   // Enable alternate function on PE2
    GPIO_PORTE_DEN_R &= ~0x04;    // Disable digital I/O on PE2
    GPIO_PORTE_AMSEL_R |= 0x04;   // Enable analog functionality on PE2
    ADC0_PC_R &= ~0xF;
    ADC0_PC_R |= 0x1;             // 125K samples/sec
    ADC0_SSPRI_R = 0x0123;        // Sequencer 3 is highest priority
    ADC0_ACTSS_R &= ~0x0008;      // Disable sample sequencer 3
    ADC0_EMUX_R &= ~0xF000;       // Seq3 is software trigger
    ADC0_SSMUX3_R &= ~0x000F;
    ADC0_SSMUX3_R += 1;           // Set channel Ain1 (PE2)
    ADC0_SSCTL3_R = 0x0006;       // No TS0 D0, yes IE0 END0
    ADC0_IM_R &= ~0x0008;         // Disable SS3 interrupts
    ADC0_ACTSS_R |= 0x0008;       // Enable sample sequencer 3
}

void Timer2A_Handler(void) {
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;  // Clear the timer interrupt

    ADC0_PSSI_R = 0x0008;               // Start ADC sample sequence 3
    while((ADC0_RIS_R & 0x08) == 0){};  // Wait for conversion to complete
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF; // Read ADC result (12-bit)
    ADC0_ISC_R = 0x0008;                // Clear completion flag

    Fifo_Put(&sampleFifo, (uint8_t)(result >> 4));  // Store 8-bit sample in FIFO
}

volatile uint32_t zero_crossings;
uint32_t simple_frequency_detection(uint16_t* buffer, uint32_t buffer_size) {
    uint32_t zero_crossings = 0;
    int16_t prev_sample = (int16_t)buffer[0] - 128;  // Convert to signed

    for (uint32_t i = 1; i < buffer_size; i++) {
        int16_t current_sample = (int16_t)buffer[i] - 128;  // Convert to signed
        if ((prev_sample < 0 && current_sample >= 0) || (prev_sample >= 0 && current_sample < 0)) {
            zero_crossings++;
        }
        prev_sample = current_sample;
    }

    // Calculate frequency: zero crossings * (sample rate / 2) / buffer size
//    return (2 * buffer_size) / (zero_crossings * SAMPLE_RATE)
    return (zero_crossings * SAMPLE_RATE) / (2 * buffer_size);
}
