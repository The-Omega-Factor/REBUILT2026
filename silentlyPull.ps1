Set-Location $PSScriptRoot

while ($true) {
    try {
        git pull origin main | Out-Null
    }
    catch {
        Add-Content "$PSScriptRoot\pull.log" "$(Get-Date) - git pull failed"
    }

    Start-Sleep -Seconds 300
}
