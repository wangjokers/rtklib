[CmdletBinding()]
param(
    [string]$DataDirectory,
    [string]$ObsPath,
    [string]$NavPath,
    [string]$B2bPath,
    [ValidateSet('auto', 'unicore', 'sino')]
    [string]$B2bFormat = 'auto',
    [string]$ConfigTemplate,
    [string]$ExecutablePath,
    [string]$Sp3Path,
    [string]$ClkPath,
    [double[]]$ReferenceEcef,
    [string]$StartTime,
    [string]$EndTime,
    [ValidateSet('M0', 'M1', 'M2', 'M3')]
    [string[]]$Modes = @('M0', 'M1', 'M2'),
    [string]$OutputRoot,
    [string]$ExperimentId,
    [ValidateRange(0, 5)]
    [int]$TraceLevel = 3,
    [ValidateRange(0.0, 1000000.0)]
    [double]$ConvergenceThresholdM = 0.10,
    [ValidateRange(1, 1000000)]
    [int]$ConvergenceHoldEpochs = 10
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

$ScriptDirectory = [System.IO.Path]::GetFullPath($PSScriptRoot)
$RtkLibRoot = [System.IO.Path]::GetFullPath((Join-Path $ScriptDirectory '..\..'))
$PythonTool = Join-Path $RtkLibRoot 'tools\pppb2b_validation\pppb2b_validation.py'
$Invariant = [System.Globalization.CultureInfo]::InvariantCulture

function Resolve-ExistingFile {
    param([string]$Path, [string]$Role)
    if ([string]::IsNullOrWhiteSpace($Path)) { return $null }
    $resolved = if ([System.IO.Path]::IsPathRooted($Path)) {
        [System.IO.Path]::GetFullPath($Path)
    }
    else {
        [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $Path))
    }
    if (-not [System.IO.File]::Exists($resolved)) {
        throw "$Role 文件不存在：$resolved"
    }
    return $resolved
}

function Find-UniqueFile {
    param(
        [string]$Root,
        [string[]]$Patterns,
        [string]$Role,
        [switch]$Optional
    )
    foreach ($pattern in $Patterns) {
        $matches = @(Get-ChildItem -LiteralPath $Root -Recurse -File -Filter $pattern -ErrorAction SilentlyContinue)
        if ($matches.Count -eq 1) { return $matches[0].FullName }
        if ($matches.Count -gt 1) {
            $paths = ($matches.FullName -join [Environment]::NewLine)
            throw "自动发现 $Role 时，模式 '$pattern' 匹配到多个文件。请显式指定路径：`n$paths"
        }
    }
    if ($Optional) { return $null }
    throw "在 $Root 下未自动发现 $Role 文件"
}

function Read-ConfigValue {
    param([string]$Path, [string]$Key)
    $pattern = '^\s*' + [regex]::Escape($Key) + '\s*=\s*(.*?)\s*(?:#.*)?$'
    foreach ($line in [System.IO.File]::ReadAllLines($Path)) {
        $match = [regex]::Match($line, $pattern)
        if ($match.Success) { return $match.Groups[1].Value.Trim() }
    }
    return $null
}

function Set-ConfigValue {
    param([string[]]$Lines, [string]$Key, [string]$Value)
    $pattern = '^\s*' + [regex]::Escape($Key) + '\s*='
    $found = $false
    $result = foreach ($line in $Lines) {
        if ([regex]::IsMatch($line, $pattern)) {
            if ($found) { throw "配置键重复出现：$Key" }
            $found = $true
            $commentIndex = $line.IndexOf('#')
            $comment = if ($commentIndex -ge 0) { ' ' + $line.Substring($commentIndex).TrimStart() } else { '' }
            "$Key=$Value$comment"
        }
        else {
            $line
        }
    }
    if (-not $found) { $result += "$Key=$Value" }
    return [string[]]$result
}

function Format-InvariantDouble {
    param([double]$Value)
    return $Value.ToString('R', $Invariant)
}

function Quote-PowerShellArgument {
    param([string]$Value)
    return "'" + $Value.Replace("'", "''") + "'"
}

function Write-Utf8NoBom {
    param([string]$Path, [string]$Content)
    [System.IO.File]::WriteAllText($Path, $Content, [System.Text.UTF8Encoding]::new($false))
}

function Invoke-PythonTool {
    param([string[]]$Arguments)
    & $script:PythonCommand $script:PythonPrefixArgs $PythonTool @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Python 验证工具失败，退出码为 $LASTEXITCODE"
    }
}

$ExperimentDirectory = $null
trap {
    $message = $_.Exception.Message
    try {
        $failureRoot = if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
            Join-Path $RtkLibRoot 'pppb2b_validation_outputs'
        }
        elseif ([System.IO.Path]::IsPathRooted($OutputRoot)) {
            [System.IO.Path]::GetFullPath($OutputRoot)
        }
        else {
            [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $OutputRoot))
        }
        $failureId = if ($ExperimentId -and $ExperimentId -match '^[A-Za-z0-9_.-]+$' -and
            -not [System.IO.Directory]::Exists((Join-Path $failureRoot $ExperimentId))) {
            $ExperimentId
        }
        else {
            'blocked_{0}_{1}' -f (Get-Date -Format 'yyyyMMdd_HHmmss'), ([guid]::NewGuid().ToString('N').Substring(0, 8))
        }
        $ExperimentDirectory = Join-Path $failureRoot $failureId
        [System.IO.Directory]::CreateDirectory($ExperimentDirectory) | Out-Null
        $blockedText = "# 阻塞报告（BLOCKED）`n`n验证总控在启动主程序前停止。`n`n- $message`n"
        [System.IO.File]::WriteAllText(
            (Join-Path $ExperimentDirectory 'BLOCKED.md'),
            $blockedText,
            [System.Text.UTF8Encoding]::new($false)
        )
        [Console]::Error.WriteLine("PPP-B2b 验证已阻塞。报告：$(Join-Path $ExperimentDirectory 'BLOCKED.md')")
    }
    catch {
        [Console]::Error.WriteLine("PPP-B2b 验证在写出报告前已阻塞：$message")
    }
    exit 2
}

$Python = Get-Command python -ErrorAction SilentlyContinue
$PythonPrefixArgs = @()
if (-not $Python) {
    $Python = Get-Command py -ErrorAction SilentlyContinue
    if ($Python) { $PythonPrefixArgs = @('-3') }
}
if (-not $Python) { throw 'PATH 中未找到 Python 3；本工具不要求安装任何全局软件包。' }
$PythonCommand = $Python.Source

if (-not [System.IO.File]::Exists($PythonTool)) {
    throw "缺少 Python 验证工具：$PythonTool"
}

if ([string]::IsNullOrWhiteSpace($ConfigTemplate)) {
    $preferredConfig = Join-Path $RtkLibRoot 'conf\sino_b2b.conf'
    if ([System.IO.File]::Exists($preferredConfig)) {
        $ConfigTemplate = $preferredConfig
    }
    else {
        $ConfigTemplate = Find-UniqueFile -Root (Join-Path $RtkLibRoot 'conf') -Patterns @('*b2b*.conf') -Role '配置模板'
    }
}
$ConfigTemplate = Resolve-ExistingFile -Path $ConfigTemplate -Role '配置模板'

if ([string]::IsNullOrWhiteSpace($B2bPath)) {
    $configuredB2b = Read-ConfigValue -Path $ConfigTemplate -Key 'file-b2brawfile'
    if ($configuredB2b -and [System.IO.File]::Exists($configuredB2b)) {
        $B2bPath = [System.IO.Path]::GetFullPath($configuredB2b)
    }
}

if ([string]::IsNullOrWhiteSpace($DataDirectory)) {
    if ($B2bPath -and [System.IO.File]::Exists($B2bPath)) {
        $DataDirectory = [System.IO.Path]::GetDirectoryName([System.IO.Path]::GetFullPath($B2bPath))
    }
    else {
        $DataDirectory = Join-Path $RtkLibRoot 'data'
    }
}
$DataDirectory = if ([System.IO.Path]::IsPathRooted($DataDirectory)) {
    [System.IO.Path]::GetFullPath($DataDirectory)
}
else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $DataDirectory))
}
if (-not [System.IO.Directory]::Exists($DataDirectory)) {
    throw "数据目录不存在：$DataDirectory"
}

if ([string]::IsNullOrWhiteSpace($ObsPath)) {
    $ObsPath = Find-UniqueFile -Root $DataDirectory -Patterns @('*_MO.rnx', '*MO.rnx', '*.obs') -Role 'RINEX 观测'
}
if ([string]::IsNullOrWhiteSpace($NavPath)) {
    $NavPath = Find-UniqueFile -Root $DataDirectory -Patterns @('*_MN.rnx', '*MN.rnx', '*.nav') -Role 'RINEX 广播导航'
}
$ObsPath = Resolve-ExistingFile -Path $ObsPath -Role '观测'
$NavPath = Resolve-ExistingFile -Path $NavPath -Role '导航'

$NeedsB2b = @($Modes | Where-Object { $_ -in @('M1', 'M2') }).Count -gt 0
if ($NeedsB2b -and [string]::IsNullOrWhiteSpace($B2bPath)) {
    $B2bPath = Find-UniqueFile -Root $DataDirectory -Patterns @('*.B2bBin', '*B2b*PPP*.txt', '*.b2b') -Role 'B2b raw'
}
if ($B2bPath) { $B2bPath = Resolve-ExistingFile -Path $B2bPath -Role 'B2b raw' }

if ('M3' -in $Modes) {
    if ([string]::IsNullOrWhiteSpace($Sp3Path)) {
        $Sp3Path = Find-UniqueFile -Root $DataDirectory -Patterns @('*.SP3', '*.sp3') -Role 'SP3' -Optional
    }
    if ([string]::IsNullOrWhiteSpace($ClkPath)) {
        $ClkPath = Find-UniqueFile -Root $DataDirectory -Patterns @('*.CLK', '*.clk') -Role 'CLK' -Optional
    }
}
if ($Sp3Path) { $Sp3Path = Resolve-ExistingFile -Path $Sp3Path -Role 'SP3' }
if ($ClkPath) { $ClkPath = Resolve-ExistingFile -Path $ClkPath -Role 'CLK' }

if ([string]::IsNullOrWhiteSpace($ExecutablePath)) {
    $candidateExecutables = @(
        (Join-Path $RtkLibRoot 'x64\Release\rnx2rtkp.exe'),
        (Join-Path $RtkLibRoot 'x64\Debug\rnx2rtkp.exe')
    )
    $ExecutablePath = $candidateExecutables | Where-Object { [System.IO.File]::Exists($_) } | Select-Object -First 1
}
$ExecutablePath = Resolve-ExistingFile -Path $ExecutablePath -Role 'rnx2rtkp 可执行程序'

if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $RtkLibRoot 'pppb2b_validation_outputs'
}
$OutputRoot = if ([System.IO.Path]::IsPathRooted($OutputRoot)) {
    [System.IO.Path]::GetFullPath($OutputRoot)
}
else {
    [System.IO.Path]::GetFullPath((Join-Path (Get-Location).Path $OutputRoot))
}
if ([string]::IsNullOrWhiteSpace($ExperimentId)) {
    $ExperimentId = '{0}_{1}' -f (Get-Date -Format 'yyyyMMdd_HHmmss'), ([guid]::NewGuid().ToString('N').Substring(0, 8))
}
if ($ExperimentId -notmatch '^[A-Za-z0-9_.-]+$') {
    throw 'ExperimentId 只能包含字母、数字、点、下划线和连字符。'
}
$ExperimentDirectory = Join-Path $OutputRoot $ExperimentId
if ([System.IO.Directory]::Exists($ExperimentDirectory) -or [System.IO.File]::Exists($ExperimentDirectory)) {
    throw "实验输出已存在，拒绝覆盖：$ExperimentDirectory"
}
[System.IO.Directory]::CreateDirectory($ExperimentDirectory) | Out-Null

try {
    $PreflightPath = Join-Path $ExperimentDirectory 'preflight.json'
    $preflightArguments = @(
        'preflight',
        '--obs', $ObsPath,
        '--nav', $NavPath,
        '--config', $ConfigTemplate,
        '--executable', $ExecutablePath,
        '--b2b-format', $B2bFormat,
        '--output', $PreflightPath
    )
    if ($B2bPath) { $preflightArguments += @('--b2b', $B2bPath) }
    if ($Sp3Path) { $preflightArguments += @('--sp3', $Sp3Path) }
    if ($ClkPath) { $preflightArguments += @('--clk', $ClkPath) }
    if ($StartTime) { $preflightArguments += @('--start', $StartTime) }
    if ($EndTime) { $preflightArguments += @('--end', $EndTime) }
    $preflightArguments += '--modes'
    $preflightArguments += $Modes
    if ($ReferenceEcef) {
        if ($ReferenceEcef.Count -ne 3) { throw 'ReferenceEcef 必须恰好包含 X、Y、Z 三个值。' }
        $preflightArguments += '--reference-ecef'
        $preflightArguments += @($ReferenceEcef | ForEach-Object { Format-InvariantDouble $_ })
    }
    Invoke-PythonTool -Arguments $preflightArguments
    $Preflight = Get-Content -LiteralPath $PreflightPath -Raw -Encoding UTF8 | ConvertFrom-Json

    $WindowStart = [datetime]::Parse($Preflight.requested_window.start_gpst, $Invariant)
    $WindowEnd = [datetime]::Parse($Preflight.requested_window.end_gpst, $Invariant)
    $StartDateArgument = $WindowStart.ToString('yyyy/MM/dd', $Invariant)
    $StartClockArgument = $WindowStart.ToString('HH:mm:ss.fff', $Invariant)
    $EndDateArgument = $WindowEnd.ToString('yyyy/MM/dd', $Invariant)
    $EndClockArgument = $WindowEnd.ToString('HH:mm:ss.fff', $Invariant)
    $ProcessingEpochs = [int]$Preflight.observation.window_epoch_count
    $ObservationInterval = if ($null -ne $Preflight.observation.interval_seconds) { [double]$Preflight.observation.interval_seconds } else { 0.0 }
    $ResolvedReference = @($Preflight.reference.ecef_xyz_m | ForEach-Object { [double]$_ })
    $ReferenceSource = [string]$Preflight.reference.source
    $DetectedB2bFormat = if ($Preflight.b2b) { [string]$Preflight.b2b.format } else { $null }

    $ModeDescriptions = @{
        M0 = '广播星历，不使用 PPP-B2b'
        M1 = '广播星历加 PPP-B2b 轨道/钟差，关闭码偏差'
        M2 = '广播星历加 PPP-B2b 轨道/钟差和码偏差'
        M3 = '精密产品 PPP 对照'
    }

    $M1OverrideKey = $null
    $M1OverrideValue = $null
    $OptionsSource = Join-Path $RtkLibRoot 'src\options.c'
    foreach ($knownKey in @('pos1-b2bcodebias', 'pos1-b2bcbias', 'pos1-b2b-code-bias')) {
        $templateHasKey = $null -ne (Read-ConfigValue -Path $ConfigTemplate -Key $knownKey)
        $sourceHasKey = [System.IO.File]::Exists($OptionsSource) -and
            [System.IO.File]::ReadAllText($OptionsSource).Contains('"' + $knownKey + '"')
        if ($templateHasKey -and $sourceHasKey) {
            $M1OverrideKey = $knownKey
            $M1OverrideValue = 'off'
            break
        }
    }

    $ReproduceParts = @(
        '&', (Quote-PowerShellArgument $PSCommandPath),
        '-ObsPath', (Quote-PowerShellArgument $ObsPath),
        '-NavPath', (Quote-PowerShellArgument $NavPath),
        '-ConfigTemplate', (Quote-PowerShellArgument $ConfigTemplate),
        '-ExecutablePath', (Quote-PowerShellArgument $ExecutablePath),
        '-StartTime', (Quote-PowerShellArgument ($WindowStart.ToString('yyyy/MM/dd HH:mm:ss.fff', $Invariant))),
        '-EndTime', (Quote-PowerShellArgument ($WindowEnd.ToString('yyyy/MM/dd HH:mm:ss.fff', $Invariant))),
        '-Modes', (($Modes | ForEach-Object { Quote-PowerShellArgument $_ }) -join ','),
        '-OutputRoot', (Quote-PowerShellArgument $OutputRoot),
        '-ConvergenceThresholdM', (Format-InvariantDouble $ConvergenceThresholdM),
        '-ConvergenceHoldEpochs', $ConvergenceHoldEpochs
    )
    if ($B2bPath) { $ReproduceParts += @('-B2bPath', (Quote-PowerShellArgument $B2bPath), '-B2bFormat', (Quote-PowerShellArgument $DetectedB2bFormat)) }
    if ($Sp3Path) { $ReproduceParts += @('-Sp3Path', (Quote-PowerShellArgument $Sp3Path)) }
    if ($ClkPath) { $ReproduceParts += @('-ClkPath', (Quote-PowerShellArgument $ClkPath)) }
    if ($ResolvedReference.Count -eq 3) {
        $ReproduceParts += @('-ReferenceEcef', (($ResolvedReference | ForEach-Object { Format-InvariantDouble $_ }) -join ','))
    }
    Write-Utf8NoBom -Path (Join-Path $ExperimentDirectory 'reproduce.ps1') -Content (($ReproduceParts -join ' ') + [Environment]::NewLine)

    $ModeResultPaths = @()
    foreach ($Mode in $Modes) {
        $ModeDirectory = Join-Path $ExperimentDirectory $Mode
        [System.IO.Directory]::CreateDirectory($ModeDirectory) | Out-Null
        $ModeResultPath = Join-Path $ModeDirectory 'mode_result.json'
        $ModeResultPaths += $ModeResultPath
        $ModeIssues = @()
        $ModeIssues += @($Preflight.blocking_issues)
        $modeIssueProperty = $Preflight.mode_issues.PSObject.Properties[$Mode]
        if ($modeIssueProperty) { $ModeIssues += @($modeIssueProperty.Value) }
        if ($Mode -eq 'M1' -and -not $M1OverrideKey) {
            $ModeIssues += '当前主程序没有在保留 PPP-B2b 轨道/钟差的同时独立关闭码偏差的配置开关；不得把 M2 改名冒充 M1。'
        }

        if ($ModeIssues.Count -gt 0) {
            $Reason = ($ModeIssues -join '; ')
            Write-Utf8NoBom -Path (Join-Path $ModeDirectory 'BLOCKED.md') -Content ("# $Mode 阻塞报告（BLOCKED）`n`n$Reason`n")
            Write-Utf8NoBom -Path (Join-Path $ModeDirectory 'command.ps1') -Content ("# 阻塞（BLOCKED）：$Reason`n")
            Write-Utf8NoBom -Path (Join-Path $ModeDirectory 'run.log') -Content ("状态：blocked`n原因：$Reason`n主程序未启动`n")
            $summaryArguments = @(
                'summarize', '--mode', $Mode, '--mode-description', $ModeDescriptions[$Mode],
                '--status', 'blocked', '--block-reason', $Reason,
                '--processing-epochs', $ProcessingEpochs.ToString($Invariant),
                '--reference-source', $ReferenceSource,
                '--run-log', (Join-Path $ModeDirectory 'run.log'),
                '--command-file', (Join-Path $ModeDirectory 'command.ps1'),
                '--output', $ModeResultPath
            )
            if ($ResolvedReference.Count -eq 3) {
                $summaryArguments += '--reference-ecef'
                $summaryArguments += @($ResolvedReference | ForEach-Object { Format-InvariantDouble $_ })
            }
            Invoke-PythonTool -Arguments $summaryArguments
            continue
        }

        $ConfigPath = Join-Path $ModeDirectory 'generated.conf'
        $ConfigLines = [System.IO.File]::ReadAllLines($ConfigTemplate)
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-solformat' -Value 'xyz'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-timesys' -Value 'gpst'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-timeform' -Value 'hms'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-outhead' -Value 'on'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-outopt' -Value 'on'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'out-outstat' -Value 'off'
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'file-tracefile' -Value (Join-Path $ModeDirectory 'trace.log')
        $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'file-solstatfile' -Value (Join-Path $ModeDirectory 'solution.stat')
        if ($Mode -in @('M0', 'M3')) {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'pos1-b2bformat' -Value 'off'
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'file-b2brawfile' -Value ''
        }
        else {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'pos1-b2bformat' -Value $DetectedB2bFormat
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'file-b2brawfile' -Value $B2bPath
        }
        if ($Mode -eq 'M3') {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'pos1-sateph' -Value 'precise'
        }
        elseif ($Mode -eq 'M0') {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'pos1-sateph' -Value 'brdc'
        }
        else {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key 'pos1-sateph' -Value 'brdc+b2b-apc'
        }
        if ($Mode -eq 'M1') {
            $ConfigLines = Set-ConfigValue -Lines $ConfigLines -Key $M1OverrideKey -Value $M1OverrideValue
        }
        [System.IO.File]::WriteAllLines($ConfigPath, $ConfigLines, [System.Text.UTF8Encoding]::new($false))

        $SolutionPath = Join-Path $ModeDirectory 'solution.pos'
        $StdoutPath = Join-Path $ModeDirectory 'run.stdout.log'
        $StderrPath = Join-Path $ModeDirectory 'run.stderr.log'
        $RunLogPath = Join-Path $ModeDirectory 'run.log'
        $CommandPath = Join-Path $ModeDirectory 'command.ps1'
        $RunArguments = @(
            '-k', $ConfigPath,
            '-o', $SolutionPath,
            '-ts', $StartDateArgument, $StartClockArgument,
            '-te', $EndDateArgument, $EndClockArgument,
            '-x', $TraceLevel.ToString($Invariant),
            $ObsPath, $NavPath
        )
        if ($Mode -eq 'M3') { $RunArguments += @($Sp3Path, $ClkPath) }
        $CommandText = '& ' + (Quote-PowerShellArgument $ExecutablePath) + ' ' + (($RunArguments | ForEach-Object { Quote-PowerShellArgument ([string]$_) }) -join ' ')
        Write-Utf8NoBom -Path $CommandPath -Content ($CommandText + [Environment]::NewLine)

        # RTKLIB writes ordinary progress (for example "reading ...") to
        # stderr. Windows PowerShell can promote redirected native stderr to
        # ErrorRecord objects when the surrounding policy is Stop, so use a
        # non-terminating policy only for the native call. The result gate
        # below still checks the real exit code, explicit "error :" markers,
        # and whether the solution format can be parsed.
        $savedErrorActionPreference = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try {
            & $ExecutablePath @RunArguments 1> $StdoutPath 2> $StderrPath
            $ExitCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $savedErrorActionPreference
        }
        $RunLog = @(
            "命令：$CommandText",
            "退出码：$ExitCode",
            '',
            '--- 标准输出（STDOUT）---',
            ([System.IO.File]::ReadAllText($StdoutPath)),
            '',
            '--- 标准错误（STDERR）---',
            ([System.IO.File]::ReadAllText($StderrPath))
        ) -join [Environment]::NewLine
        Write-Utf8NoBom -Path $RunLogPath -Content $RunLog

        $TracePath = "$SolutionPath.trace"
        $summaryArguments = @(
            'summarize', '--mode', $Mode, '--mode-description', $ModeDescriptions[$Mode],
            '--status', 'ran', '--exit-code', $ExitCode.ToString($Invariant),
            '--processing-epochs', $ProcessingEpochs.ToString($Invariant),
            '--reference-source', $ReferenceSource,
            '--experiment-start', ($WindowStart.ToString('yyyy/MM/dd HH:mm:ss.fff', $Invariant)),
            '--observation-interval', (Format-InvariantDouble $ObservationInterval),
            '--convergence-threshold', (Format-InvariantDouble $ConvergenceThresholdM),
            '--convergence-hold', $ConvergenceHoldEpochs.ToString($Invariant),
            '--config', $ConfigPath, '--solution', $SolutionPath,
            '--run-log', $RunLogPath, '--command-file', $CommandPath,
            '--output', $ModeResultPath
        )
        if ([System.IO.File]::Exists($TracePath)) { $summaryArguments += @('--trace', $TracePath) }
        if ($ResolvedReference.Count -eq 3) {
            $summaryArguments += '--reference-ecef'
            $summaryArguments += @($ResolvedReference | ForEach-Object { Format-InvariantDouble $_ })
        }
        Invoke-PythonTool -Arguments $summaryArguments
    }

    $aggregateArguments = @('aggregate', '--experiment-dir', $ExperimentDirectory, '--preflight', $PreflightPath)
    foreach ($path in $ModeResultPaths) { $aggregateArguments += @('--mode-result', $path) }
    Invoke-PythonTool -Arguments $aggregateArguments

    $Summary = Get-Content -LiteralPath (Join-Path $ExperimentDirectory 'summary.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    Write-Output "PPP-B2b 验证状态：$($Summary.overall_status)"
    Write-Output "实验目录：$ExperimentDirectory"
    Write-Output "汇总 CSV：$(Join-Path $ExperimentDirectory 'summary.csv')"
    Write-Output "实验报告：$(Join-Path $ExperimentDirectory 'experiment_report.md')"
}
catch {
    $message = $_.Exception.Message
    $blockedPath = Join-Path $ExperimentDirectory 'BLOCKED.md'
    Write-Utf8NoBom -Path $blockedPath -Content ("# 阻塞报告（BLOCKED）`n`n验证总控已停止，且没有伪造结果。`n`n- $message`n")
    [Console]::Error.WriteLine("PPP-B2b 验证已阻塞。报告：$blockedPath")
    [Console]::Error.WriteLine($message)
    exit 2
}
