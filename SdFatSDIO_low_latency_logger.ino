/*
 * Distributed under the 2-clause BSD licence (See LICENCE.TXT file at the
 * repository root).
 *
 * Copyright (c) 2017 Tilo Nitzsche.  All rights reserved.
 *
 * https://github.com/tni/teensy-samples
 */

// Sample code for data logging. It uses a pre-allocated, pre-erased
// file to minimize latency from SD card writes. This can reduce
// the worst case write latency from 850ms down to 40ms, drastically
// reducing the required buffer size.
// 
// It is intented for usage with SdFat beta:
// https://github.com/greiman/SdFat-beta
//
// with the Teensy 3.5 or 3.6 SDIO slot.
//
// The log data is captured in an ISR, triggered by an 'IntervalTimer'.
// An interrupt-safe FIFO is used for interrupt-safe communication
// with the main loop which is responsible for writing the captured data
// to the SD card.

#include <array>
#include <atomic>
#include <SdFat.h>

// Content for debugging purposes.
// Replace with something useful.
struct LogEntry {
    uint32_t counter;
    uint32_t record_offset;
    uint32_t time;
    uint32_t dummy;
};

// Buffer that holds our log entries. When a buffer is full,
// it is written to SD.
template<class entry_t, size_t entry_count>
class LogBuffer {
public:
    void addEntry(const entry_t& entry) {
        if(size < capacity) log_entries[size++] = entry;
    }
    bool full() { return size >= capacity; }
    
    static_assert(std::is_same<uint8_t, unsigned char>::value, "Aliasing rule violation.");
    uint8_t* rawData() { return (uint8_t*) log_entries.data(); }
    
    size_t rawSize() { return size * sizeof(entry_t); }
    void reset() { *this = {}; }
private:
    static constexpr size_t capacity = entry_count;
    std::array<entry_t, capacity> log_entries = {};
    size_t size = 0;
};

// Simple lock free single-producer, single-consumer FIFO. Interrupt-safe.
// It is intended for small entries.
// Note: atomic_signal_fence is a compiler barrier only. It requires a CPU
//       with memory order guarantees (like ARM Cortex M) to be sufficient.
template<class entry_t, size_t capacity_>
class FifoSpSc {
public:
    // Add entry to end of FIFO. FIFO must not be full.
    void push(const entry_t& elem) {
        atomic_signal_fence(std::memory_order_acq_rel);
        const size_t write_pos_ = write_pos;
        buffer[write_pos_] = elem;
        atomic_signal_fence(std::memory_order_acq_rel);
        write_pos = nextPos(write_pos_);
        atomic_signal_fence(std::memory_order_acq_rel);
    }
    // Remove and return first FIFO entry. FIFO must not be empty.
    entry_t pop() {
        atomic_signal_fence(std::memory_order_acq_rel);
        const size_t read_pos_ = read_pos;
        entry_t ret = buffer[read_pos_];
        atomic_signal_fence(std::memory_order_acq_rel);
        read_pos = nextPos(read_pos_);
        atomic_signal_fence(std::memory_order_acq_rel);
        return ret;
    }
    bool empty() const { atomic_signal_fence(std::memory_order_acq_rel); return read_pos == write_pos; }
    bool full() const { atomic_signal_fence(std::memory_order_acq_rel); return read_pos == nextPos(write_pos); }
    size_t size() const {
        atomic_signal_fence(std::memory_order_acq_rel);
        const size_t read_pos_ = read_pos;
        const size_t write_pos_ = write_pos;
        return write_pos_ - read_pos_ + (write_pos_ >= read_pos_ ? 0 : buffer_size);
    }
    static size_t capacity() { return capacity_; }
private:
    static size_t nextPos(size_t pos) { return pos + 1 < buffer_size ? pos + 1 : 0; }

    static constexpr size_t buffer_size = capacity_ + 1;
    std::array<entry_t, buffer_size> buffer;
    volatile size_t write_pos;
    volatile size_t read_pos;
};

// Class for pre-allocated, pre-erased file. 
// It has an internal buffer that holds partial sector writes. This allows efficient writes
// of any size.
template<class sd_fat_t, class debuglog_t = void>
class PreallocatedFile {
public:
    enum Error {
        E_ok = 0,              // no error
        E_eof,                 // end of file reached
        E_create_contiguous,   // error allocating contiguous file
        E_erase,               // error erasing allocated block
        E_write_block,         // error writing data block
    };

    // Use SdFatSdio or SdFatSdioEX for sd_fat_t.
    // 'debuglog' is optionally used for printing debug messages. Use one of the Arduino Stream classes,
    // e.g. 'Serial'.
    PreallocatedFile(sd_fat_t& sd_fat, debuglog_t* debuglog = nullptr) : sd_fat(sd_fat), debuglog(debuglog) {}
    ~PreallocatedFile() { if(next_sector) close(); }
    
    // Create a pre-allocated, pre-erased contiguous file. Any existing file 
    // with the same name, is erased.
    // Return true on success.
    bool create(const char* name, uint32_t size) {
        log("Creating file: %s\n", name);
        if(next_sector) close();
        last_error = E_ok;
        sd_fat.remove(name);
        if(!file.createContiguous(sd_fat.vwd(), name, size)) {
            log("ContigFile: createContiguous() failed");
            last_error = E_create_contiguous;
            return false;
        }
        uint32_t first_sector, last_sector;
        if(!file.contiguousRange(&first_sector, &last_sector)) {
            log("PreallocatedFile: contiguousRange() failed");
            last_error = E_create_contiguous;
            return false;
        }
        uint32_t first_erase_sector = first_sector;
        const size_t erase_count = 64 * 1024; // number of sectors to erase at once
        while(first_erase_sector <= last_sector) {
            if(!sd_fat.card()->erase(first_erase_sector, std::min(first_erase_sector+erase_count, last_sector))) {
                log("PreallocatedFile: erase() failed");
                last_error = E_erase;
                return false;
            }
            first_erase_sector += erase_count;
        }
        log("First sector: %u     last sector: %u\n", first_sector, last_sector);
        this->first_sector = first_sector;
        this->last_sector = last_sector;
        next_sector = first_sector;

        file.flush();
        return true;
    }
    
    // Only full sectors can be written to disk. Partial sectors are automatically
    // buffered. Writes of any size are allowed.
    // sync_blocks == true closes the current multi-block write and allows the SD card to go
    // to sleep. Performance is much lower.
    bool write(const uint8_t* buffer, size_t size, bool sync_blocks = false) {
        last_error = E_ok;
        if(!next_sector || next_sector > last_sector) {
            last_error = E_eof;
            return false;
        }
        if(partial_sector_len) {
            size_t copy_len = std::min(512 - partial_sector_len, size);
            memcpy(partial_sector_buffer + partial_sector_len, buffer, copy_len);
            partial_sector_len += copy_len;
            if(partial_sector_len < 512) return true;
            if(!sd_fat.card()->writeBlocks(next_sector, partial_sector_buffer, 1)) {
                last_error = E_write_block;
                log("Error writing log file. Current sector: %u\n", next_sector);
            }            
            next_sector++;
            buffer += copy_len;
            size -= copy_len;
            partial_sector_len = 0;
        }
        const size_t sector_count = size / 512;
        if(sector_count) {
            if(!sd_fat.card()->writeBlocks(next_sector, buffer, sector_count)) {
                last_error = E_write_block;
                log("Error writing log file. Current sector: %u\n", next_sector);
            }
            next_sector += sector_count;
            size -= sector_count * 512;
            buffer += sector_count * 512;
        }
        if(size) {
            memcpy(partial_sector_buffer, buffer, size);
            partial_sector_len = size;
        }
        if(sync_blocks) sd_fat.card()->syncBlocks();
        return last_error == E_ok;
    }
    
    // Buffered data is written to disk, the file is truncated to the amount of data
    // that was written and then closed.
    void close() {
        last_error = E_ok;
        if(next_sector && partial_sector_len) {
            if(!sd_fat.card()->writeBlocks(next_sector, partial_sector_buffer, 1)) {
                last_error = E_write_block;
                log("Error writing log file. Current sector: %u\n", next_sector);
            }
        }
        if(next_sector) file.truncate((next_sector - first_sector) * 512 + partial_sector_len);
        file.close();
        partial_sector_len = 0;
        first_sector = 0;
        last_sector = 0;
        next_sector = 0;
    }
    
    size_t getWriteSector() { return next_sector; }
    Error getLastError() { return last_error; }

private:
    void log(const char* msg) { if(debuglog) debuglog->println(msg); }
    
    template<class... args_t>
    void log(args_t... args) { if(debuglog) debuglog->printf(args...); }
    
    File file;
    sd_fat_t& sd_fat;
    debuglog_t* debuglog = nullptr;
    uint32_t first_sector = 0;
    uint32_t last_sector = 0;
    uint32_t next_sector = 0;
    uint8_t partial_sector_buffer[512] __attribute__ ((aligned (4))) = {};
    size_t partial_sector_len = 0;
    Error last_error = E_ok;
};

// Store 'LogEntry's in 'LogBuffer'. Each buffer holds around 4200 bytes.
using MyLogBuffer = LogBuffer<LogEntry, 4200 / sizeof(LogEntry)>;

constexpr size_t log_buffer_count = 40;
const char* log_file_name = "log.bin";
uint8_t sensor_pin = A0;
uint32_t log_file_size = 2ull * 1024 * 1024 * 1024;  // 2GB
// PIT timer interval for invoking our data capture ISR.
uint32_t capture_interval = 10;  // in microseconds

// Allocate storage for log buffers.
std::array<MyLogBuffer, log_buffer_count> log_buffers;
// FIFO that holds pointers our empty log_buffers.
FifoSpSc<MyLogBuffer*, log_buffer_count> empty_log_buffers;
// Put all allocated log buffers into 'empty_log_buffers' FIFO.
char empty_log_buffers_init = [](){ 
    for(size_t i = 0; i < log_buffer_count; i++) empty_log_buffers.push(&log_buffers[i]);
    return 0;
}();
// Filled log buffer FIFO.
FifoSpSc<MyLogBuffer*, log_buffer_count> filled_log_buffers;

auto& serial = Serial;
// Count the number of times our log buffers overflow.
std::atomic<size_t> buffer_overflow_counter;


SdFatSdioEX sd_fat;
bool logging_finished = false;
size_t written_bytes = 0;
size_t max_buffers_used = 0;


// Add log entry to log buffers.
void logEntry(const LogEntry& log_entry) {
    static MyLogBuffer* log_buffer = nullptr;
    if(!log_buffer) {
        if(empty_log_buffers.empty()) {
            buffer_overflow_counter++;
            return;
        } else {
            log_buffer = empty_log_buffers.pop();
        }
    }
    log_buffer->addEntry(log_entry);
    if(log_buffer->full()) {
        filled_log_buffers.push(log_buffer);
        log_buffer = nullptr;
    }
}

// ISR, periodically triggered by PIT timer. It creates some dummy
// log entries. Put your data capturing here.
void captureData() {
    uint32_t time = micros();
    static uint32_t counter = 0;
    static uint32_t record_offset = 0;
    
    logEntry( { counter, record_offset, time, 0x42424242 } );
    counter++;
    record_offset += sizeof(LogEntry);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    serial.begin(115200);
    delay(2000);
    serial.println("Initializing SD.");

    if (!sd_fat.begin()) {
        serial.println("SD init failed.");
        return;
    }
    serial.println("SD init done.");

    using file_t = PreallocatedFile<decltype(sd_fat), std::remove_reference<decltype(serial)>::type>;
    file_t file(sd_fat, &serial);
    file.create(log_file_name, log_file_size);
    serial.println("Log file allocated and pre-erased.");
    serial.println("\nPress 'c' to terminate logging.\n");

    elapsedMillis report_timer;
    IntervalTimer data_capture_timer;
    data_capture_timer.begin(captureData, capture_interval);

    while(true) {
        // If we have a filled log buffer, write it to SD.
        if(!filled_log_buffers.empty()) {
            max_buffers_used = std::max(max_buffers_used, empty_log_buffers.capacity() - empty_log_buffers.size());
            MyLogBuffer* log_buffer = filled_log_buffers.pop();
            if(!file.write(log_buffer->rawData(), log_buffer->rawSize())) {
                if(file.getLastError() == file.E_eof) {
                    file.close();
                    serial.println("Log file full. Logging finished.");
                    return;                    
                }
                // ignore other write errors, hopefully subsequent writes will succeed
            }
            written_bytes += log_buffer->rawSize();
            *log_buffer = {};
            empty_log_buffers.push(log_buffer);
        }
        if(report_timer > 5000) {
            serial.printf("elapsed: %us     bytes written: %u     buffer overflow count: %u\n", 
                           millis() / 1000, written_bytes, buffer_overflow_counter.load());
            serial.printf("next write sector: %u    max buffers used during last 5s: %u\n", 
                          file.getWriteSector(), max_buffers_used);
            report_timer = 0;
            max_buffers_used = 0;
        }
        if(serial.available()) {
            if(serial.read() == 'c') {
                file.close();
                serial.println("Logging finished.");
                return;
            }
        }
    }
}

// Running when an error is encountered or when logging is finished.
void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
}
