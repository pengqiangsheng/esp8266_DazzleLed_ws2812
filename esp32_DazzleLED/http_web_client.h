WebServer server(80);

const char* controlIndex = "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'><meta http-equiv='X-UA-Compatible' content='IE=edge'><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=0'><title>MQTT Web Client</title><style>#pre-loading{display: flex;justify-content: center;align-items: center;position: absolute;top: 0;left: 0;z-index: 1001;height: 100%;width: 100%;background-color: rgba(0, 0, 0, 0.7);}#pre-loading .loading-wrapper{display: flex;justify-content: center;align-items: center;}#pre-loading .loading-wrapper .block{width: 120px;height: 120px;border-radius: 6px;background-color: #fff;display: flex;justify-content: center;align-items: center;color: #c8c9cc;}#pre-loading .spinner{position: relative;display: inline-block;width: 30px;max-width: 100%;height: 30px;max-height: 100%;vertical-align: middle;-webkit-animation: van-rotate 0.8s linear infinite;animation: van-rotate 0.8s linear infinite;animation-duration: 2s;color: #c8c9cc;}#pre-loading .spinner .loading__circular{display: block;width: 100%;height: 100%;}#pre-loading .spinner .loading__circular circle{animation: van-circular 1.5s ease-in-out infinite;stroke: currentColor;stroke-width: 3;stroke-linecap: round;}</style><link href='http://img.pqs.guozhaoxi.top/mqtt_web_client_static_chunk-vendors.2eed8ddd.css' rel='stylesheet'><link href='http://img.pqs.guozhaoxi.top/mqtt_web_client_static_app.8952b2a9.css' rel='stylesheet'></head><body><div id='pre-loading'><div class='loading-wrapper'><div class='block'><div><div style='text-align: center;margin-bottom: 6px;'><span class='spinner'><svg viewBox='25 25 50 50' class='loading__circular'><circle cx='50' cy='50' r='20' fill='none'></circle></svg></span></div><div>加载中</div></div></div></div></div><div id='app'></div><script>(function(e){function t(t){for(var n,o,i=t[0],c=t[1],l=t[2],s=0,f=[];s<i.length;s++)o=i[s],Object.prototype.hasOwnProperty.call(a,o)&&a[o]&&f.push(a[o][0]),a[o]=0;for(n in c)Object.prototype.hasOwnProperty.call(c,n)&&(e[n]=c[n]);p&&p(t);while(f.length)f.shift()();return u.push.apply(u,l||[]),r()}function r(){for(var e,t=0;t<u.length;t++){for(var r=u[t],n=!0,o=1;o<r.length;o++){var i=r[o];0!==a[i]&&(n=!1)}n&&(u.splice(t--,1),e=c(c.s=r[0]))}return e}var n={},o={runtime:0},a={runtime:0},u=[];function i(e){return c.p+'http://img.pqs.guozhaoxi.top/mqtt_web_client_static_'+({}[e]||e)+'.'+{'chunk-e84e0878':'f099b889'}[e]+'.js'}function c(t){if(n[t])return n[t].exports;var r=n[t]={i:t,l:!1,exports:{}};return e[t].call(r.exports,r,r.exports,c),r.l=!0,r.exports}c.e=function(e){var t=[],r={'chunk-e84e0878':1};o[e]?t.push(o[e]):0!==o[e]&&r[e]&&t.push(o[e]=new Promise((function(t,r){for(var n='http://img.pqs.guozhaoxi.top/mqtt_web_client_static_'+({}[e]||e)+'.'+{'chunk-e84e0878':'b15f96ac'}[e]+'.css',a=c.p+n,u=document.getElementsByTagName('link'),i=0;i<u.length;i++){var l=u[i],s=l.getAttribute('data-href')||l.getAttribute('href');if('stylesheet'===l.rel&&(s===n||s===a))return t()}var f=document.getElementsByTagName('style');for(i=0;i<f.length;i++){l=f[i],s=l.getAttribute('data-href');if(s===n||s===a)return t()}var p=document.createElement('link');p.rel='stylesheet',p.type='text/css',p.onload=t,p.onerror=function(t){var n=t&&t.target&&t.target.src||a,u=new Error('Loading CSS chunk  failed.()');u.code='CSS_CHUNK_LOAD_FAILED',u.request=n,delete o[e],p.parentNode.removeChild(p),r(u)},p.href=a;var d=document.getElementsByTagName('head')[0];d.appendChild(p)})).then((function(){o[e]=0})));var n=a[e];if(0!==n)if(n)t.push(n[2]);else{var u=new Promise((function(t,r){n=a[e]=[t,r]}));t.push(n[2]=u);var l,s=document.createElement('script');s.charset='utf-8',s.timeout=120,c.nc&&s.setAttribute('nonce',c.nc),s.src=i(e);var f=new Error;l=function(t){s.onerror=s.onload=null,clearTimeout(p);var r=a[e];if(0!==r){if(r){var n=t&&('load'===t.type?'missing':t.type),o=t&&t.target&&t.target.src;f.message='Loading chunk  failed.',f.name='ChunkLoadError',f.type=n,f.request=o,r[1](f)}a[e]=void 0}};var p=setTimeout((function(){l({type:'timeout',target:s})}),12e4);s.onerror=s.onload=l,document.head.appendChild(s)}return Promise.all(t)},c.m=e,c.c=n,c.d=function(e,t,r){c.o(e,t)||Object.defineProperty(e,t,{enumerable:!0,get:r})},c.r=function(e){'undefined'!==typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:'Module'}),Object.defineProperty(e,'__esModule',{value:!0})},c.t=function(e,t){if(1&t&&(e=c(e)),8&t)return e;if(4&t&&'object'===typeof e&&e&&e.__esModule)return e;var r=Object.create(null);if(c.r(r),Object.defineProperty(r,'default',{enumerable:!0,value:e}),2&t&&'string'!=typeof e)for(var n in e)c.d(r,n,function(t){return e[t]}.bind(null,n));return r},c.n=function(e){var t=e&&e.__esModule?function(){return e['default']}:function(){return e};return c.d(t,'a',t),t},c.o=function(e,t){return Object.prototype.hasOwnProperty.call(e,t)},c.p='',c.oe=function(e){throw console.error(e),e};var l=window['webpackJsonp']=window['webpackJsonp']||[],s=l.push.bind(l);l.push=t,l=l.slice();for(var f=0;f<l.length;f++)t(l[f]);var p=s;r()})([]);</script><script src='http://img.pqs.guozhaoxi.top/mqtt_web_client_static_chunk-vendors.fa49c8da.js'></script><script src='http://img.pqs.guozhaoxi.top/mqtt_web_client_static_app.98200cc0.js'></script></body></html>";