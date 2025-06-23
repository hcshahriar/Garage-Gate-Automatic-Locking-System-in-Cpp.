#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <cstdlib>
#include "SerialPort.h" // You'll need a serial communication library

// Constants
constexpr int AUTO_LOCK_DELAY_SEC = 10;
constexpr int SENSOR_POLL_INTERVAL_MS = 500;
constexpr int SERIAL_BAUD_RATE = 9600;

// Global state
enum class GateState { LOCKED, UNLOCKED, ERROR };
GateState currentState = GateState::UNLOCKED;
bool motionDetected = false;
bool manualOverride = false;
std::mutex stateMutex;
SerialPort* arduinoPort = nullptr;

// Function declarations
void initializeHardware();
void closeHardware();
void sendLockCommand();
void sendUnlockCommand();
void monitorSensors();
void processUserInput();
void displayStatus();
void autoLockTimer();
bool isAuthorized();
void handleError(const std::string& errorMsg);

int main() {
    try {
        std::cout << "Initializing Garage Gate Control System..." << std::endl;
        
        initializeHardware();
        
        // Start sensor monitoring thread
        std::thread sensorThread(monitorSensors);
        
        // Start auto-lock timer thread
        std::thread autoLockThread(autoLockTimer);
        
        // Start user input thread
        std::thread inputThread(processUserInput);
        
        // Main loop for status display
        while (true) {
            displayStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        // Cleanup (normally won't reach here)
        sensorThread.join();
        autoLockThread.join();
        inputThread.join();
        closeHardware();
        
    } catch (const std::exception& e) {
        handleError(std::string("System failure: ") + e.what());
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

// Hardware initialization
void initializeHardware() {
    try {
        // Initialize serial connection to Arduino/Raspberry Pi
        arduinoPort = new SerialPort("COM3"); // Adjust port as needed
        if (!arduinoPort->isConnected()) {
            throw std::runtime_error("Failed to connect to Arduino");
        }
        
        // Set initial state
        sendUnlockCommand();
        
        std::cout << "Hardware initialized successfully" << std::endl;
    } catch (const std::exception& e) {
        handleError(std::string("Hardware initialization failed: ") + e.what());
        throw;
    }
}

// Clean up hardware resources
void closeHardware() {
    if (arduinoPort != nullptr) {
        delete arduinoPort;
        arduinoPort = nullptr;
    }
}

// Send lock command to hardware
void sendLockCommand() {
    std::lock_guard<std::mutex> lock(stateMutex);
    
    if (currentState == GateState::LOCKED) return;
    
    try {
        if (arduinoPort->writeSerialPort("LOCK", 4) != 4) {
            throw std::runtime_error("Failed to send lock command");
        }
        
        // Wait for acknowledgment
        char ack[4] = {0};
        if (arduinoPort->readSerialPort(ack, 3) < 3 || std::string(ack) != "ACK") {
            throw std::runtime_error("No acknowledgment from hardware");
        }
        
        currentState = GateState::LOCKED;
        std::cout << "Gate locked successfully" << std::endl;
    } catch (const std::exception& e) {
        currentState = GateState::ERROR;
        handleError(std::string("Locking failed: ") + e.what());
    }
}

// Send unlock command to hardware
void sendUnlockCommand() {
    std::lock_guard<std::mutex> lock(stateMutex);
    
    if (currentState == GateState::UNLOCKED) return;
    
    try {
        if (arduinoPort->writeSerialPort("UNLK", 4) != 4) {
            throw std::runtime_error("Failed to send unlock command");
        }
        
        // Wait for acknowledgment
        char ack[4] = {0};
        if (arduinoPort->readSerialPort(ack, 3) < 3 || std::string(ack) != "ACK") {
            throw std::runtime_error("No acknowledgment from hardware");
        }
        
        currentState = GateState::UNLOCKED;
        std::cout << "Gate unlocked successfully" << std::endl;
    } catch (const std::exception& e) {
        currentState = GateState::ERROR;
        handleError(std::string("Unlocking failed: ") + e.what());
    }
}

// Monitor sensor inputs
void monitorSensors() {
    while (true) {
        try {
            // Request sensor status
            if (arduinoPort->writeSerialPort("SENS", 4) != 4) {
                throw std::runtime_error("Failed to request sensor status");
            }
            
            // Read sensor response
            char sensorData[5] = {0};
            if (arduinoPort->readSerialPort(sensorData, 4) < 4) {
                throw std::runtime_error("Failed to read sensor data");
            }
            
            // Update motion detection status
            bool newMotion = (std::string(sensorData) == "MOT1");
            
            std::lock_guard<std::mutex> lock(stateMutex);
            motionDetected = newMotion;
            
            // If motion detected and gate is locked, check authorization and unlock
            if (motionDetected && currentState == GateState::LOCKED && !manualOverride) {
                if (isAuthorized()) {
                    sendUnlockCommand();
                }
            }
            
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lock(stateMutex);
            currentState = GateState::ERROR;
            handleError(std::string("Sensor monitoring error: ") + e.what());
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_POLL_INTERVAL_MS));
    }
}

// Automatic locking timer
void autoLockTimer() {
    auto lastMotionTime = std::chrono::steady_clock::now();
    
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        std::lock_guard<std::mutex> lock(stateMutex);
        
        if (currentState == GateState::ERROR || manualOverride) {
            continue;
        }
        
        if (motionDetected) {
            lastMotionTime = std::chrono::steady_clock::now();
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - lastMotionTime).count();
                
            if (elapsed >= AUTO_LOCK_DELAY_SEC && currentState == GateState::UNLOCKED) {
                sendLockCommand();
            }
        }
    }
}

// Process user input for manual control
void processUserInput() {
    while (true) {
        std::cout << "\nEnter command (lock/unlock/status/exit): ";
        std::string input;
        std::getline(std::cin, input);
        
        std::lock_guard<std::mutex> lock(stateMutex);
        
        if (input == "lock") {
            manualOverride = true;
            sendLockCommand();
        } else if (input == "unlock") {
            manualOverride = true;
            sendUnlockCommand();
        } else if (input == "status") {
            displayStatus();
        } else if (input == "exit") {
            std::cout << "Exiting system..." << std::endl;
            closeHardware();
            exit(EXIT_SUCCESS);
        } else if (input == "auto") {
            manualOverride = false;
            std::cout << "Manual override disabled, returning to automatic mode" << std::endl;
        } else {
            std::cout << "Invalid command" << std::endl;
        }
    }
}

// Display current system status
void displayStatus() {
    std::lock_guard<std::mutex> lock(stateMutex);
    
    system("cls"); // or "clear" for Linux
    
    std::cout << "=== Garage Gate Control System ===" << std::endl;
    std::cout << "Current State: ";
    
    switch (currentState) {
        case GateState::LOCKED:
            std::cout << "LOCKED";
            break;
        case GateState::UNLOCKED:
            std::cout << "UNLOCKED";
            break;
        case GateState::ERROR:
            std::cout << "ERROR";
            break;
    }
    
    std::cout << "\nMotion Detected: " << (motionDetected ? "YES" : "NO") << std::endl;
    std::cout << "Control Mode: " << (manualOverride ? "MANUAL" : "AUTO") << std::endl;
    std::cout << "\nCommands: lock, unlock, status, auto, exit" << std::endl;
}

// Check for authorization (simplified - expand for real use)
bool isAuthorized() {
    // In a real system, this would check RFID, keypad, etc.
    // For now, we'll just return true as a placeholder
    return true;
}

// Error handling
void handleError(const std::string& errorMsg) {
    std::cerr << "\n!!! ERROR: " << errorMsg << " !!!" << std::endl;
    
    // Attempt to put system in safe state
    try {
        if (arduinoPort != nullptr && arduinoPort->isConnected()) {
            arduinoPort->writeSerialPort("STOP", 4);
        }
    } catch (...) {
        // Ignore secondary errors during error handling
    }
}
