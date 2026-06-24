#!/usr/bin/env python3
"""
ns-3-ai Python Agent for RET Optimization
Integrates with Samsung LLM Agent API for antenna tilt decisions

Usage:
    python3 ret-ai.py

Environment Variables:
    SAMSUNG_API_KEY: Your Samsung agent API key (optional, uses placeholder if not set)
"""

import ns3ai_gym_env # type: ignore
import gymnasium as gym
import sys
import traceback
import os
import json
import requests
import re

# ── Configuration ─────────────────────────────────────────────────────────────
API_URL = "https://agent.sec.samsung.net/api/v1/run/4bd4a876-d330-4e88-8e83-cf9029201b51?stream=false"
API_KEY = os.environ.get("SAMSUNG_API_KEY", "sk-Dprx376aoTr_SEo8HwSnXMPG6wcMs6X8Go4RAebdILs")
SESSION_ID = "4bd4a876-d330-4e88-8e83-cf9029201b51"

# Observation format: time, total, avg, min, p5, max, jfi, coverage, tilt0, tilt1, tilt2
NUM_OBS_FEATURES = 11

# Action format: 3 tilt angles (0-15 degrees)
NUM_GNB = 3
TILT_MIN = 0
TILT_MAX = 15


class RetAiAgent:
    """Agent that calls Samsung LLM API for tilt decisions."""

    def __init__(self):
        self.api_key = API_KEY
        self.api_url = API_URL
        self.request_count = 0
        print(f"[Agent] Initialized with API URL: {self.api_url}")
        if self.api_key == "YOUR_API_KEY_HERE":
            print("[WARNING] Using placeholder API key. Set SAMSUNG_API_KEY env var.")

    def format_prompt(self, obs):
        """
        Format observation into a prompt for the LLM.
        
        Observation format (11 features):
        [time, total_tput, avg_tput, min_tput, p5_tput, max_tput, jfi, coverage, tilt0, tilt1, tilt2]
        """
        time_s = obs[0]
        total_tput = obs[1]
        avg_tput = obs[2]
        min_tput = obs[3]
        p5_tput = obs[4]
        max_tput = obs[5]
        jfi = obs[6]
        coverage = obs[7]
        tilt0 = obs[8]
        tilt1 = obs[9]
        tilt2 = obs[10]

        # Format as ret_summary line (matches C++ output format)
        ret_summary_line = f"{time_s:.1f},{total_tput:.2f},{avg_tput:.2f},{min_tput:.2f},{p5_tput:.2f},{max_tput:.2f},{jfi:.3f},{coverage:.1f},{int(tilt0)},{int(tilt1)},{int(tilt2)}"

        prompt = f"""Current network state (ret_summary format: time_s,total_tput_Mbps,avg_tput_Mbps,min_tput_Mbps,p5_tput_Mbps,max_tput_Mbps,jfi,coverage_pct,gnb0_tilt,gnb1_tilt,gnb2_tilt):
{ret_summary_line}

Recommend optimal antenna tilt angles (0-15 degrees) for the 3 gNBs to maximize total throughput and fairness (JFI).
Consider:
- Higher coverage % means more UEs are satisfied
- Higher JFI means more fair resource distribution
- Current tilts are: gNB0={int(tilt0)}°, gNB1={int(tilt1)}°, gNB2={int(tilt2)}°

Respond with ONLY three comma-separated integers for new tilt angles (e.g., 5,8,7)."""

        return prompt

    def parse_response(self, response_text):
        """
        Parse LLM response to extract 3 tilt angles.
        Uses regex to find 3 comma-separated integers.
        """
        # Try to find pattern like "5,8,7" or "5, 8, 7"
        pattern = r'(\d+)\s*,\s*(\d+)\s*,\s*(\d+)'
        match = re.search(pattern, response_text)
        
        if match:
            tilt0 = int(match.group(1))
            tilt1 = int(match.group(2))
            tilt2 = int(match.group(3))
            
            # Clamp to valid range
            tilt0 = max(TILT_MIN, min(TILT_MAX, tilt0))
            tilt1 = max(TILT_MIN, min(TILT_MAX, tilt1))
            tilt2 = max(TILT_MIN, min(TILT_MAX, tilt2))
            
            return [tilt0, tilt1, tilt2]
        
        # Fallback: return current tilts (no change)
        print(f"[WARNING] Could not parse response: '{response_text}'. Using default [7,7,7]")
        return [7, 7, 7]

    def call_api(self, prompt):
        """
        Call Samsung LLM API with the prompt.
        Returns the response text.
        """
        self.request_count += 1
        
        headers = {
            "Content-Type": "application/json",
            "x-api-key": self.api_key
        }
        
        payload = {
            "input_type": "chat",
            "output_type": "chat",
            "input_value": prompt
        }
        
        try:
            response = requests.post(
                self.api_url,
                headers=headers,
                json=payload,
                timeout=30  # 30 second timeout
            )
            response.raise_for_status()
            
            result = response.json()
            
            # Parse response structure:
            # {"outputs":[{"inputs":{"input_value":"..."},"outputs":[{"results":{"message":{"text":"..."}}}]}]}
            text = result["outputs"][0]["outputs"][0]["results"]["message"]["text"]
            
            return text
            
        except requests.exceptions.Timeout:
            print(f"[ERROR] API request timed out (attempt {self.request_count})")
            return None
        except requests.exceptions.RequestException as e:
            print(f"[ERROR] API request failed: {e}")
            return None
        except (KeyError, IndexError, json.JSONDecodeError) as e:
            print(f"[ERROR] Failed to parse API response: {e}")
            print(f"Raw response: {response.text if 'response' in locals() else 'N/A'}")
            return None

    def get_action(self, obs, reward, done, info):
        """
        Get action from Samsung LLM API.
        
        Args:
            obs: Observation array (11 features)
            reward: Current reward value
            done: Whether episode is done
            info: Additional info dict
        
        Returns:
            List of 3 tilt angles [tilt0, tilt1, tilt2]
        """
        # Format prompt with current observation
        prompt = self.format_prompt(obs)
        
        print(f"\n[Agent] === Decision Request #{self.request_count + 1} ===")
        print(f"[Agent] Observation: time={obs[0]:.1f}s, total={obs[1]:.2f}Mbps, JFI={obs[6]:.3f}, coverage={obs[7]:.1f}%")
        print(f"[Agent] Current tilts: [{int(obs[8])}, {int(obs[9])}, {int(obs[10])}]")
        
        # Call API
        response_text = self.call_api(prompt)
        
        if response_text is None:
            print("[Agent] API call failed, using fallback action [7,7,7]")
            return [7, 7, 7]
        
        print(f"[Agent] LLM response: '{response_text.strip()}'")
        
        # Parse response to get tilts
        action = self.parse_response(response_text)
        
        print(f"[Agent] Selected action: {action}")
        print(f"[Agent] ========================================\n")
        
        return action


def main():
    """Main entry point."""
    print("=" * 60)
    print("ns-3-ai RET Optimization Agent with Samsung LLM API")
    print("=" * 60)
    
    # Create environment
    print("\n[Init] Creating Gymnasium environment...")
    env = gym.make(
        "ns3ai_gym_env/Ns3-v0",
        targetName="ret-ai",  # Match source file name (scratch/ret-ai.cc)
        ns3Path="./"         # ns-3 root is current directory (run from ns-3.46/)
    )
    
    ob_space = env.observation_space
    ac_space = env.action_space
    
    print(f"[Init] Observation space: {ob_space} (dtype: {ob_space.dtype})")
    print(f"[Init] Action space: {ac_space} (dtype: {ac_space.dtype})")
    
    # Create agent
    agent = RetAiAgent()
    
    try:
        # Reset environment (starts ns-3 simulation)
        print("\n[Init] Resetting environment (starting ns-3 simulation)...")
        obs, info = env.reset()
        print(f"[Init] Initial observation: {obs}")
        
        reward = 0.0
        done = False
        step_count = 0
        
        print("\n[Run] Starting decision loop...")
        print("=" * 60)
        
        while not done:
            step_count += 1
            
            # Get action from LLM agent
            action = agent.get_action(obs, reward, done, info)
            
            # Send action to ns-3
            obs, reward, done, truncated, info = env.step(action)
            
            print(f"[Step {step_count}] Reward: {reward:.3f}, Done: {done}")
            
            if truncated:
                print("[Run] Environment truncated, exiting loop")
                break
        
        print("\n[Run] Episode completed!")
        
    except Exception as e:
        print(f"\n[ERROR] Exception occurred: {e}")
        print("Traceback:")
        traceback.print_tb(sys.exc_info()[2])
        sys.exit(1)
    
    finally:
        print("\n[Shutdown] Closing environment...")
        env.close()
        print("[Shutdown] Environment closed")
    
    print("\n" + "=" * 60)
    print(f"Summary: {step_count} decisions made")
    print(f"API calls: {agent.request_count}")
    print("=" * 60)


if __name__ == "__main__":
    main()
