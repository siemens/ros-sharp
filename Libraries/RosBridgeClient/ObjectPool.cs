/*
© University of Kent, 2019
Author: Odysseas Doumas <od79@kent.ac.uk>
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Collections.Concurrent;

namespace RosSharp.RosBridgeClient
{
    internal class ObjectPool<Tout> where Tout : class
    {
        [ThreadStatic]
        private static Tout _fastCache;

        private ConcurrentBag<Tout> _objects;
        private Func<Tout> _objectGenerator;

        public ObjectPool(Func<Tout> objectGenerator)
        {
            _objects = new ConcurrentBag<Tout>();
            _objectGenerator = objectGenerator ?? throw new ArgumentNullException("objectGenerator");
        }

        public Tout GetObject()
        {
            Tout item;
            if (_fastCache != null)
            {
                item = _fastCache;
                _fastCache = null;
                return item;
            }
            if (_objects.TryTake(out item)) return item;
            return _objectGenerator();
        }

        public void PutObject(Tout item)
        {
            if (_fastCache == null)
                _fastCache = item;
            else
                _objects.Add(item);
        }
    }
}
