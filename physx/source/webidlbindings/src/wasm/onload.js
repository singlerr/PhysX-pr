/**
 * Makes the API a little less verbose
 */
Object.defineProperty(PhysX, 'PHYSICS_VERSION', { get() { return PhysX.PxTopLevelFunctions.prototype.PHYSICS_VERSION; }});

//Move PxTopLevelFunctions to PhysX object
for(const prop in PhysX.PxTopLevelFunctions.prototype) {
    if(prop !== 'constructor' && !prop.startsWith('get_') && !prop.startsWith('__')) {
        Object.defineProperty(PhysX, prop, { get() { return PhysX.PxTopLevelFunctions.prototype[prop]; }});
    }
}

//Move NativeArrayHelpers to PhysX object
for(const prop in PhysX.NativeArrayHelpers.prototype) {
    if(prop !== 'constructor' && !prop.startsWith('get_') && !prop.startsWith('__')) {
        Object.defineProperty(PhysX, prop, { get() { return PhysX.NativeArrayHelpers.prototype[prop]; }});
    }
}

//Group enums
const regex = /_emscripten_enum_(.*?)_(.*)/;
const enums = Object.keys(PhysX).filter(key => key.includes('_emscripten_enum_')).map(emscript => emscript.match(regex));

for (const [emscript, enumName, entryName] of enums) {
    PhysX[enumName] ??= {};
    Object.defineProperty(PhysX[enumName], entryName, { get() { return PhysX[emscript](); }});
}