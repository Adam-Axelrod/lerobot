from lerobot.common.policies.factory import make_policy
import torch

ckpt_path = "outputs/train/act_experiment/checkpoints/020000/pretrained_model"
try:
    policy = make_policy(checkpoint_path=ckpt_path)
    print("✅ Policy loaded successfully!")
except Exception as e:
    print(f"❌ Failed to load policy: {e}")