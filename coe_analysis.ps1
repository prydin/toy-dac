$raw = Get-Content "C:\proj\toy-day\data\interpolator.coe" -Raw
$data = ($raw -split 'coefdata=')[1] -replace ';',''
$coeffs = $data.Trim() -split '\s+'
$count = $coeffs.Count
$max = 0
$min = 0
foreach ($c in $coeffs) {
    $v = [long]$c
    if ($v -gt $max) { $max = $v }
    if ($v -lt $min) { $min = $v }
}
"Total coefficients: $count"
"Max: $max"
"Min: $min"
"Bits needed (unsigned max): $([Math]::Ceiling([Math]::Log([Math]::Abs($max)+1, 2)))"
"Bits needed (signed): $([Math]::Ceiling([Math]::Log([Math]::Abs($min)+1, 2)) + 1)"

# Check for symmetry
$sym = $true
for ($i=0; $i -lt [Math]::Floor($count/2); $i++) {
    if ([long]$coeffs[$i] -ne [long]$coeffs[$count-1-$i]) { $sym = $false; break }
}
"Symmetric: $sym"

# Check if 1100 / 100 = 11 taps per polyphase
"Taps per polyphase phase (1100/100): $($count / 100)"

# Sum of all coefficients
$sum = [double]0
foreach ($c in $coeffs) { $sum += [double]$c }
"Sum all: $sum"
"log2(sum): $([Math]::Log([Math]::Abs($sum), 2))"

# Sum of every 100th coeff (one polyphase phase)
for ($p=0; $p -lt 3; $p++) {
    $psum = [double]0
    for ($t=0; $t -lt 11; $t++) {
        $idx = $p + $t * 100
        if ($idx -lt $count) { $psum += [double]$coeffs[$idx] }
    }
    "Phase $p sum: $psum"
}
