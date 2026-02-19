while ($true) {
    # Go to your repository folder
    Set-Location "C:\Users\haole\Programs\FRC\2026\REBUILT2026"

    # Pull latest changes
    git pull origin main

    # Wait 60 seconds before running again
    Start-Sleep -Seconds 60
}