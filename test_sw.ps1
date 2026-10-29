try {
    $sw = [System.Runtime.InteropServices.Marshal]::GetActiveObject('SldWorks.Application')
    Write-Output "Connected to SolidWorks: $($sw.RevisionNumber())"
    $model = $sw.IActiveDoc2
    if ($model -ne $null) {
        Write-Output "Active Doc Title: $($model.GetTitle())"
        Write-Output "Doc Type: $($model.GetType())"
    } else {
        Write-Output "No active doc."
    }
} catch {
    Write-Output "Error: $_"
}

