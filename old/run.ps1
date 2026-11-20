$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    & "$PSScriptRoot\pnr\Scripts\python.exe" `
        "$PSScriptRoot\run_batch.py" `
        --maze-seed 42 `
        --seed 42 `
        --workers 2 `
        --max-steps 200 `
        --gpu 0 `
        @args

    if ($LASTEXITCODE -ne 0) {
        throw "Batch runner exited with code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}