function Test-Checkpoint {
    param(
        [Parameter(Mandatory=$true)][string]$Ckpt,
        [string]$Run = "dish_red_dot_new_4",
        [string]$Task = "dish_red_dot",
        [string]$User = "AdamAxelrod",
        [Nullable[double]]$EnsembleCoeff = $null
    )
    # Note: ACT only supports two sensible inference modes -
    #   1. Full chunks (default, n_action_steps == chunk_size): fast but wobbles at chunk boundaries.
    #   2. Temporal ensembling (n_action_steps=1, coeff>0): smooth but every-step inference.
    # Values of n_action_steps strictly between 1 and chunk_size always re-execute the early-ramp
    # of each chunk, causing the robot to stutter. Don't override n_action_steps without ensembling.
    $repo = "$User/eval_${Run}_${Ckpt}"
    Remove-Item "$env:USERPROFILE\.cache\huggingface\lerobot\$repo" -Recurse -Force -ErrorAction SilentlyContinue

    $args = @(
        "--robot.type=meca500"
        "--robot.monitor_mode=False"
        "--teleop.type=meca500_home"
        "--teleop.home=[0,0,0,0,90,0]"
        "--display_data=true"
        "--dataset.repo_id=$repo"
        "--dataset.single_task=$Task"
        "--dataset.push_to_hub=false"
        "--policy.path=outputs\train\$Run\checkpoints\$Ckpt\pretrained_model"
    )
    if ($null -ne $EnsembleCoeff) {
        $args += "--policy.temporal_ensemble_coeff=$EnsembleCoeff"
        $args += "--policy.n_action_steps=1"
    }
    lerobot-record @args
}

# . .\inference.ps1
# Test-Checkpoint -Ckpt "005000"
# Test-Checkpoint -Ckpt "005000" -EnsembleCoeff 0.01
# $env:PYTHONWARNINGS = "ignore::UserWarning:torchvision.io._video_deprecation_warning"

