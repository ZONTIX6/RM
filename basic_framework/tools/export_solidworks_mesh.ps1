param(
    [Parameter(Mandatory=$true)]
    [string]$InputPath,

    [Parameter(Mandatory=$true)]
    [string]$OutputPath
)

$ErrorActionPreference = "Stop"

$swDocPART = 1
$swDocASSEMBLY = 2
$swOpenDocOptions_Silent = 1
$swSaveAsCurrentVersion = 0
$swSaveAsOptions_Silent = 1

$inputFull = [System.IO.Path]::GetFullPath($InputPath)
$outputFull = [System.IO.Path]::GetFullPath($OutputPath)
$outputDir = [System.IO.Path]::GetDirectoryName($outputFull)
if (-not [System.IO.Directory]::Exists($outputDir)) {
    [System.IO.Directory]::CreateDirectory($outputDir) | Out-Null
}

$ext = [System.IO.Path]::GetExtension($inputFull).ToLowerInvariant()
$docType = if ($ext -eq ".sldasm") { $swDocASSEMBLY } else { $swDocPART }

$sw = $null
foreach ($progId in @("SldWorks.Application.34", "SldWorks.Application")) {
    try {
        Write-Output "trying=$progId"
        $sw = New-Object -ComObject $progId
        break
    } catch {
        Write-Output "failed=$progId message=$($_.Exception.Message)"
    }
}
if ($null -eq $sw) {
    throw "Could not create SolidWorks COM application."
}

Write-Output "solidworks_started"

$errors = 0
$warnings = 0
Write-Output "opening=$inputFull"
$model = $sw.OpenDoc6($inputFull, $docType, $swOpenDocOptions_Silent, "", [ref]$errors, [ref]$warnings)
if ($null -eq $model) {
    throw "SolidWorks OpenDoc6 failed. errors=$errors warnings=$warnings input=$inputFull"
}

$saveErrors = 0
$saveWarnings = 0
Write-Output "saving=$outputFull"
$ok = $model.Extension.SaveAs($outputFull, $swSaveAsCurrentVersion, $swSaveAsOptions_Silent, $null, [ref]$saveErrors, [ref]$saveWarnings)
if (-not $ok) {
    throw "SolidWorks SaveAs failed. errors=$saveErrors warnings=$saveWarnings output=$outputFull"
}

$sw.CloseDoc($model.GetTitle())
$sw.ExitApp()

Write-Output "exported=$outputFull"
