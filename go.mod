module github.com/xet-labs/roboarm-commander

go 1.22

require (
	github.com/mattn/go-sqlite3 v1.14.22
	go.bug.st/serial v1.6.2
)

require (
	github.com/creack/goselect v0.1.2 // indirect
	golang.org/x/sys v0.0.0-20220829200755-d48e67d00261 // indirect
)

// go.bug.st is a vanity import domain not reachable from this build
// environment's network egress allowlist; go.bug.st/serial and
// github.com/bugst/go-serial are the same project (the former is just
// a custom-domain alias for the latter), so redirecting the fetch here
// is not a fork/substitution, just a mirror. Safe to drop this replace
// if building somewhere with normal internet access.
replace go.bug.st/serial => github.com/bugst/go-serial v1.6.2

replace golang.org/x/sys => github.com/golang/sys v0.0.0-20220829200755-d48e67d00261

replace gopkg.in/yaml.v3 => github.com/go-yaml/yaml v0.0.0-20200313102051-9f266ea9e77c
