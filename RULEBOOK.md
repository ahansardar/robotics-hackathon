# **TURTLEBOT PURSUIT & EVASION CHALLENGE**

###  **Rule Book**

**Chase. Evade. Outthink.**

---

## **1\. COMPETITION OBJECTIVE**

The competition challenges teams to develop autonomous algorithms for two roles:

* **Catcher:** Pursue and capture the Runner in the shortest possible time.  
* **Runner:** Evade the Catcher and survive for the duration of the match.

---

## **2\. COMPETITION STRUCTURE**

### **ROUND 1 — CATCHER QUALIFICATION**

* Teams compete using their **Catcher algorithm**.  
* A standardized moving target acts as the Runner.  
* Performance is based on **successful capture time**.  
* If more than 8 teams qualify, the **8 fastest teams** advance to Round 2\.

### **ROUND 2 — 1v1 KNOCKOUT**

Each matchup consists of **two innings of 3 minutes each**:

| Inning | Catcher | Runner |
| ----- | ----- | ----- |
| 1 | Team A | Team B |
| 2 | Team B | Team A |

Both teams are evaluated in both roles.

---

## **3\. ARENA SPECIFICATIONS**

| Parameter | Specification |
| ----- | ----- |
| Arena | 10 m × 10 m enclosed 2D arena |
| Obstacles | 4–6 static obstacles |
| Initial distance | 3 m |
| Head start | None |
| Starting condition | Both robots start simultaneously |
| Maximum inning duration | 3 minutes |

---

## **4\. CAPTURE RULE**

A Runner is considered **captured** only when:

**Distance ≤ 0.5 m for a continuous 1 second.**

* A momentary distance of ≤0.5 m does not count.  
* The capture time is recorded once the condition is successfully met.

---

## **5\. WIN CONDITIONS**

### **CATCHER WIN**

The Catcher successfully captures the Runner within 3 minutes.

**Shorter capture time \= better performance.**

### **RUNNER WIN**

The Catcher fails to capture the Runner within 3 minutes.

The final Catcher–Runner distance may be recorded for tie-breaking.

---

## **6\. TIE-BREAKING**

If required, the following order will be used:

1. **Both Catchers capture:** Faster capture time wins.  
2. **Both Runners evade:** Greater final distance wins.  
3. **Only one team captures:** The team achieving the capture receives the advantage.  
4. **Further tie:** An additional standardized tie-breaker may be conducted.

---

## **7\. ALGORITHM REQUIREMENTS**

### **CATCHER ALGORITHM**

The algorithm should support:

* Runner detection and tracking  
* Autonomous navigation  
* Obstacle avoidance  
* Pursuit planning  
* Real-time trajectory adaptation  
* Efficient capture

### **RUNNER ALGORITHM**

The algorithm should support:

* Autonomous navigation  
* Obstacle avoidance  
* Escape planning  
* Response to Catcher movement  
* Continuous strategy adaptation  
* Survival for the full 3-minute inning

---

## **8\. SIMULATION ENVIRONMENT**

* Competition will be conducted in a **simulated TurtleBot environment**.  
* The platform is tentatively **TurtleBot 4 Lite**.  
* Technical implementation details will be communicated separately by the organizing committee.

---

## **9\. STANDARDIZATION RULES**

All teams will compete under the same conditions:

* Fixed arena dimensions  
* 3 m initial distance  
* No head start  
* Simultaneous start  
* 4–6 static obstacles  
* Maximum 3-minute inning  
* 0.5 m capture threshold  
* 1-second capture requirement  
* Standardized moving target in Round 1

**10.SUBMISSION**

1. **Round 1 :** Documentation via GitHub repository (including a comprehensive *readme.md*) and video evidence.  
2. **Round 2 :** Participation in an offline knockout format featuring 3-minute innings where Team A and Team B deploy custom hunter and catcher algorithms in direct competition.

## **11\. ORGANIZING COMMITTEE**

The organizing committee reserves the right to modify the competition structure when required based on the number of qualifiers or competition requirements.

Decisions regarding **qualification, match outcomes, tie-breaking, and competition structure shall be final**. Any rule or technical updates will be communicated officially.

### **12.Competition Schedule**

* **Round 1 – Online:** 12th & 13th September  
* **Round 2 – Offline:** 30th October & 1st November

