#include <AGVCoreNetwork.h>
#include <AGVCoreMotion.h>

// 1. Command handler for AGVCoreNetwork (Core 0)
void onCommandReceived(const char* command, uint8_t source, uint8_t priority) {
  // Forward ALL commands to AGVCoreMotion (Core 1 will filter them)
  agvMotion.processCommand(command, source, priority);
  
  // Also send status back to interfaces
  String status = "Received: ";
  status += command;
  agvNetwork.sendStatus(status.c_str());
}

// 2. Movement status callback (optional)
void onMovementStatus(const char* status) {
  agvNetwork.sendStatus(String("MOTION: ") + status);
}

// 3. Emergency callback (optional)
void onEmergencyStop() {
  agvNetwork.broadcastEmergency("MOTION SYSTEM EMERGENCY STOP");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   AGV DUAL-CORE SYSTEM v1.0           ║");
  Serial.println("║   Core 0: AGVCoreNetwork              ║");
  Serial.println("║   Core 1: AGVCoreMotion               ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Initialize Core 0 communication system - LINE 1
  agvNetwork.begin("factory_agv_01", "admin", "agv_secure_pass");
  
  // Register command callback to forward commands to Core 1 - LINE 2
  agvNetwork.setCommandCallback(onCommandReceived);
  
  // Initialize Core 1 motion system - LINE 3
  agvMotion.begin();
  
  // Configure callbacks and optional settings - LINE 4
  agvMotion.setMovementCallback(onMovementStatus);
  agvMotion.setEmergencyCallback(onEmergencyStop);
  // agvMotion.setWheelConfig(200, 600); // Optional custom configuration
  // agvMotion.setSpeedConfig(65, 50, 5); // Optional custom configuration
  
  Serial.println("\n✅ AGV Dual-Core System Ready!");
  Serial.println("🌐 Web interface: http://factory_agv_01.local");
  Serial.println("⌨️  Serial commands: move forward, turn_left 90, STOP, PATH:1,1,3,2:ONCE");
  Serial.println("📱 Connect to 'AGV_Controller_Network' for initial WiFi setup");
}

void loop() {
  // NOTHING NEEDED HERE!
  // Core 0 handles all communication
  // Core 1 handles all motor control
  // They communicate through thread-safe queues
  
  // Optional: Send periodic status update
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    agvNetwork.sendStatus("AGV System: All Cores Operational");
    lastStatus = millis();
  }
  
  delay(100);
}