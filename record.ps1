function Start-Recording {
    param(
        [Parameter(Mandatory=$true)][string]$Name,
        [Parameter(Mandatory=$true)][string]$Task,
        [int]$NumEpisodes = 50,
        [int]$Fps = 30,
        [int]$EpisodeTimeSeconds = 60,
        [int]$ResetTimeSeconds = 60,
        [string]$User = "AdamAxelrod",
        [switch]$NoDisplay
    )
    $repo = "$User/$Name"
    Remove-Item "$env:USERPROFILE\.cache\huggingface\lerobot\$repo" -Recurse -Force -ErrorAction SilentlyContinue

    $display = if ($NoDisplay) { "false" } else { "true" }

    $args = @(
        "--robot.type=meca500"
        "--teleop.type=meca500_bota"
        "--dataset.repo_id=$repo"
        "--dataset.single_task=$Task"
        "--dataset.num_episodes=$NumEpisodes"
        "--dataset.fps=$Fps"
        "--dataset.episode_time_s=$EpisodeTimeSeconds"
        "--dataset.reset_time_s=$ResetTimeSeconds"
        "--display_data=$display"
    )
    lerobot-record @args
}

# . .\record.ps1
# Start-Recording -Name "red_dot" -Task "reach_red_dot" -NumEpisodes 50
