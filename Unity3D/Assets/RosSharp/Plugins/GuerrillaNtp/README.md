# GuerrillaNtp #

GuerrillaNtp is a simple NTP (SNTP) client written in C# that can be embedded in desktop .NET applications
to provide them with accurate network time even when the system clock is unsynchronized.

```csharp
// query the SNTP server
TimeSpan offset;
using (var ntp = new NtpClient(Dns.GetHostAddresses("pool.ntp.org")[0]))
    offset = ntp.GetCorrectionOffset();

// use the offset throughout your app
var accurateTime = DateTime.UtcNow + offset;
```

* [Homepage](https://guerrillantp.machinezoo.com/) - Overview, download, tutorial, alternatives, contact.
* [Documentation](https://guerrillantp.machinezoo.com/api/) - API reference documentation.
* [Sources](https://bitbucket.org/robertvazan/guerrillantp/src) - Primary repository, preferred for pull requests.
* [License](https://www.apache.org/licenses/LICENSE-2.0) - Distributed under Apache License 2.0.

