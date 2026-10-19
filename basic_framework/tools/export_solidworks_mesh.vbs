Option Explicit

Dim inputPath, outputPath
If WScript.Arguments.Count < 2 Then
    WScript.Echo "usage: cscript export_solidworks_mesh.vbs input.sldasm output.stl"
    WScript.Quit 2
End If

inputPath = WScript.Arguments(0)
outputPath = WScript.Arguments(1)

Dim sw, model, errors, warnings, saveErrors, saveWarnings, ok
On Error Resume Next
Set sw = CreateObject("SldWorks.Application.34")
If Err.Number <> 0 Then
    WScript.Echo "create versioned failed: " & Err.Description
    Err.Clear
    Set sw = CreateObject("SldWorks.Application")
End If
If Err.Number <> 0 Or sw Is Nothing Then
    WScript.Echo "create failed: " & Err.Description
    WScript.Quit 1
End If
On Error GoTo 0

errors = 0
warnings = 0
WScript.Echo "opening=" & inputPath
On Error Resume Next
Set model = sw.OpenDoc(inputPath, 2)
If Err.Number <> 0 Then
    WScript.Echo "OpenDoc failed: " & Err.Description
    Err.Clear
    Set model = sw.OpenDoc6(inputPath, 2, 1, "", errors, warnings)
End If
If Err.Number <> 0 Then
    WScript.Echo "OpenDoc6 failed: " & Err.Description
    WScript.Quit 1
End If
On Error GoTo 0
If model Is Nothing Then
    WScript.Echo "open failed errors=" & errors & " warnings=" & warnings
    WScript.Quit 1
End If

saveErrors = 0
saveWarnings = 0
WScript.Echo "saving=" & outputPath
On Error Resume Next
ok = model.SaveAs(outputPath)
If Err.Number <> 0 Or Not ok Then
    WScript.Echo "SaveAs failed: " & Err.Description
    Err.Clear
    ok = model.Extension.SaveAs(outputPath, 0, 1, Nothing, saveErrors, saveWarnings)
End If
If Err.Number <> 0 Then
    WScript.Echo "Extension.SaveAs failed: " & Err.Description
    WScript.Quit 1
End If
On Error GoTo 0
If Not ok Then
    WScript.Echo "save failed errors=" & saveErrors & " warnings=" & saveWarnings
    WScript.Quit 1
End If

sw.CloseDoc model.GetTitle()
sw.ExitApp
WScript.Echo "exported=" & outputPath
