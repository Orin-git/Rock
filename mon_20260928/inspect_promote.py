from json import JSONDecoder
t = open("/tmp/prod_promote.log").read()
i = t.find("{")
d, _ = JSONDecoder().raw_decode(t[i:])
print("top-level keys:", list(d))
for k in ("reload", "rollback_test", "rollback", "promote", "RELOADING", "reloaded"):
    if k in d:
        print(f"--- {k} ---")
        print(("  " + str(d[k]))[:800])
